"""Render a small Markdown research report to Korean ODT, without flight I/O."""
import argparse
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET
from zipfile import ZipFile, ZIP_DEFLATED, ZIP_STORED

NS = {p: f'urn:oasis:names:tc:opendocument:xmlns:{p}:1.0'
      for p in ('office', 'style', 'text', 'table', 'meta', 'manifest')}
NS.update(fo='urn:oasis:names:tc:opendocument:xmlns:xsl-fo-compatible:1.0',
          svg='urn:oasis:names:tc:opendocument:xmlns:svg-compatible:1.0',
          xlink='http://www.w3.org/1999/xlink', dc='http://purl.org/dc/elements/1.1/')
for prefix, uri in NS.items():
    ET.register_namespace(prefix, uri)


def q(name):
    prefix, local = name.split(':', 1)
    return f'{{{NS[prefix]}}}{local}'


def add(parent, name, attrs=None, text=None):
    node = ET.SubElement(parent, q(name), {q(k): str(v) for k, v in (attrs or {}).items()})
    node.text = text
    return node


def append_text(parent, value):
    if len(parent):
        parent[-1].tail = (parent[-1].tail or '') + value
    else:
        parent.text = (parent.text or '') + value


def fonts(parent):
    for name, family in [('Korean', 'Noto Sans CJK KR'), ('Mono', 'Noto Sans Mono CJK KR')]:
        add(parent, 'style:font-face', {'style:name': name, 'svg:font-family': family})


class Document:
    def __init__(self, references):
        self.references, self.cited, self.table_count = references, set(), 0
        self.root = ET.Element(q('office:document-content'), {q('office:version'): '1.2'})
        fonts(add(self.root, 'office:font-face-decls'))
        self.auto = add(self.root, 'office:automatic-styles')
        self.body = add(add(self.root, 'office:body'), 'office:text')

    def inline(self, parent, value, citations=True):
        value = re.sub(r'(\[\^\d+\])(?=\[\^)', r'\1,', value)
        for token in re.split(r'(\[\^\d+\]|\*\*.*?\*\*|`[^`]+`|https?://[^\s]+)', value):
            if not token:
                continue
            if re.fullmatch(r'\[\^\d+\]', token) and citations:
                number = int(token[2:-1])
                ref = self.references[number]
                if number in self.cited:
                    add(parent, 'text:a', {'xlink:type': 'simple', 'xlink:href': f'#source_{number}',
                                          'text:style-name': 'Superscript'}, str(number))
                else:
                    self.cited.add(number)
                    note = add(parent, 'text:note', {'text:id': f'note_{number}', 'text:note-class': 'footnote'})
                    add(note, 'text:note-citation', {'text:label': str(number)}, str(number))
                    p = add(add(note, 'text:note-body'), 'text:p', {'text:style-name': 'Footnote'})
                    quoted = re.search(r'[「“](.*?)[」”]', ref)
                    author = ref.split(',')[0]
                    title = quoted.group(1) if quoted else ref[len(author)+2:].split(' . ')[0][:95]
                    p.text = author + ', ' + title.rstrip(', .') + '. '
                    url = re.search(r'https?://\S+', ref)
                    if url:
                        add(p, 'text:a', {'xlink:type': 'simple', 'xlink:href': url.group(0)}, '원문')
                    else:
                        append_text(p, '로컬 기록. 출처 목록 참조.')
            elif token.startswith('**'):
                add(parent, 'text:span', {'text:style-name': 'Bold'}, token[2:-2])
            elif token.startswith('`'):
                add(parent, 'text:span', {'text:style-name': 'InlineCode'}, token[1:-1])
            elif token.startswith(('https://', 'http://')):
                add(parent, 'text:a', {'xlink:type': 'simple', 'xlink:href': token}, token)
            else:
                append_text(parent, token)

    def paragraph(self, value, style='Body', parent=None):
        p = add(self.body if parent is None else parent, 'text:p', {'text:style-name': style})
        self.inline(p, value)
        return p

    def heading(self, value, level, pagebreak=False):
        style = 'Title' if level == 1 else ('Chapter' if level == 2 else 'Subheading')
        if pagebreak:
            style += 'Break'
        p = add(self.body, 'text:h', {'text:style-name': style, 'text:outline-level': str(level)})
        self.inline(p, value)

    def code(self, lines):
        for line in lines:
            p = add(self.body, 'text:p', {'text:style-name': 'Code'})
            for part in re.split(r'( +)', line):
                if part.startswith(' '):
                    add(p, 'text:s', {'text:c': str(len(part))})
                else:
                    append_text(p, part)
        self.paragraph('', 'After')

    def table(self, rows):
        self.table_count += 1
        cols = len(rows[0])
        if any(len(r) != cols for r in rows):
            raise ValueError('Mismatched table cells')
        ratios = {2: [.30, .70], 3: [.25, .36, .39], 4: [.23, .27, .29, .21]}[cols]
        name = f'Table{self.table_count}'
        table = add(self.body, 'table:table', {'table:name': name, 'table:style-name': 'DataTable'})
        for i, ratio in enumerate(ratios):
            cname = f'{name}C{i}'
            style = add(self.auto, 'style:style', {'style:name': cname, 'style:family': 'table-column'})
            add(style, 'style:table-column-properties', {'style:column-width': f'{17*ratio:.3f}cm'})
            add(table, 'table:table-column', {'table:style-name': cname})
        for index, row in enumerate(rows):
            target = add(table, 'table:table-header-rows') if index == 0 else table
            tr = add(target, 'table:table-row', {'table:style-name': 'TableRow'})
            for cell in row:
                tc = add(tr, 'table:table-cell', {'table:style-name': 'HeaderCell' if index == 0 else 'DataCell',
                                                'office:value-type': 'string'})
                self.paragraph(cell, 'TableHead' if index == 0 else 'TableText', tc)
        self.paragraph('', 'After')

    def markdown(self, value):
        lines = value.splitlines()
        i, pagebreak = 0, False
        while i < len(lines):
            line = lines[i].strip()
            if not line:
                i += 1
                continue
            if line == '<!-- pagebreak -->':
                pagebreak = True
                i += 1
                continue
            if re.match(r'#{1,3} ', line):
                level = len(line)-len(line.lstrip('#'))
                self.heading(line[level+1:], level, pagebreak)
                pagebreak = False
                i += 1
                continue
            if line.startswith('```'):
                block = []
                i += 1
                while i < len(lines) and not lines[i].strip().startswith('```'):
                    block.append(lines[i])
                    i += 1
                self.code(block)
                i += 1
                continue
            if line.startswith('|'):
                rows = []
                while i < len(lines) and lines[i].strip().startswith('|'):
                    cells = [c.strip() for c in lines[i].strip().strip('|').split('|')]
                    if not all(re.fullmatch(r':?-+:?', c) for c in cells):
                        rows.append(cells)
                    i += 1
                self.table(rows)
                continue
            para = [line]
            i += 1
            while i < len(lines) and lines[i].strip() and not lines[i].lstrip().startswith(('#', '|', '```', '<!--')):
                para.append(lines[i].strip())
                i += 1
            self.paragraph(' '.join(para))

    def sources(self):
        for index, number in enumerate(sorted(self.references)):
            if index % 10 == 0:
                self.heading('출처' if index == 0 else '출처 — 계속', 2, True)
                if index == 0:
                    self.paragraph('공식 문서·원 논문·로컬 기록을 우선 사용했다. 온라인 열람 기준일은 2026-09-11이다. 이동하는 문서는 실제 설치 버전과 구분했다.', 'Small')
            p = add(self.body, 'text:p', {'text:style-name': 'Source'})
            add(p, 'text:bookmark', {'text:name': f'source_{number}'})
            self.inline(p, f'[{number}] {self.references[number]}', False)

    def styles_xml(self):
        root = ET.Element(q('office:document-styles'), {q('office:version'): '1.2'})
        fonts(add(root, 'office:font-face-decls'))
        styles = add(root, 'office:styles')
        default = add(styles, 'style:default-style', {'style:family': 'paragraph'})
        add(default, 'style:paragraph-properties', {'fo:line-height': '145%', 'fo:orphans': '2', 'fo:widows': '2'})
        add(default, 'style:text-properties', {'style:font-name': 'Korean', 'style:font-name-asian': 'Korean',
            'fo:font-size': '10.5pt', 'style:font-size-asian': '10.5pt', 'fo:language': 'ko', 'fo:country': 'KR',
            'style:language-asian': 'ko', 'style:country-asian': 'KR', 'fo:color': '#202124'})
        specs = {'Body': (10.5,145,0,.20,False), 'Title': (24,125,0,.55,True),
                 'Chapter': (17,130,.40,.32,True), 'ChapterBreak': (17,130,0,.32,True),
                 'Subheading': (12,135,.32,.15,True), 'TableText': (9.1,135,0,0,False),
                 'TableHead': (9.1,135,0,0,True), 'Footnote': (7.1,125,0,.06,False),
                 'Code': (8.4,132,0,0,False), 'After': (3,100,0,.14,False),
                 'Small': (9,140,0,.25,False), 'Source': (8.8,140,0,.35,False)}
        for name, (size, line, top, bottom, bold) in specs.items():
            s = add(styles, 'style:style', {'style:name': name, 'style:family': 'paragraph'})
            pp = {'fo:line-height': f'{line}%', 'fo:margin-top': f'{top}cm', 'fo:margin-bottom': f'{bottom}cm',
                  'fo:orphans': '2', 'fo:widows': '2'}
            if name.endswith('Break'):
                pp['fo:break-before'] = 'page'
            if name in ('Title', 'Chapter', 'ChapterBreak', 'Subheading'):
                pp['fo:keep-with-next'] = 'always'
            if name == 'Source':
                pp['fo:keep-together'] = 'always'
            add(s, 'style:paragraph-properties', pp)
            tp = {'fo:font-size': f'{size}pt', 'style:font-size-asian': f'{size}pt'}
            if bold:
                tp.update({'fo:font-weight': 'bold', 'style:font-weight-asian': 'bold'})
            if name == 'Code':
                tp.update({'style:font-name': 'Mono', 'style:font-name-asian': 'Mono'})
            add(s, 'style:text-properties', tp)
        for name, props in {'Bold': {'fo:font-weight':'bold', 'style:font-weight-asian':'bold'},
                            'Superscript': {'style:text-position':'super 70%'},
                            'InlineCode': {'style:font-name':'Mono', 'style:font-name-asian':'Mono'}}.items():
            add(add(styles, 'style:style', {'style:name':name, 'style:family':'text'}), 'style:text-properties', props)
        s = add(styles, 'style:style', {'style:name':'DataTable', 'style:family':'table'})
        add(s, 'style:table-properties', {'style:width':'17cm', 'table:align':'left'})
        s = add(styles, 'style:style', {'style:name':'TableRow', 'style:family':'table-row'})
        add(s, 'style:table-row-properties', {'fo:keep-together':'always'})
        for name, bg in [('DataCell', '#ffffff'), ('HeaderCell', '#ededed')]:
            s = add(self.auto, 'style:style', {'style:name':name, 'style:family':'table-cell'})
            add(s, 'style:table-cell-properties', {'fo:padding':'.06cm', 'fo:border':'.4pt solid #b7b7b7',
                                                'fo:background-color':bg, 'style:vertical-align':'middle'})
        add(styles, 'text:notes-configuration', {'text:note-class':'footnote', 'text:default-style-name':'Footnote',
            'style:num-format':'1', 'text:start-numbering-at':'document', 'text:footnotes-position':'page'})
        page = add(add(root, 'office:automatic-styles'), 'style:page-layout', {'style:name':'A4'})
        add(page, 'style:page-layout-properties', {'fo:page-width':'21cm', 'fo:page-height':'29.7cm',
            'style:print-orientation':'portrait', 'fo:margin-top':'1.8cm', 'fo:margin-bottom':'1.8cm',
            'fo:margin-left':'2cm', 'fo:margin-right':'2cm'})
        add(add(root, 'office:master-styles'), 'style:master-page', {'style:name':'Standard', 'style:page-layout-name':'A4'})
        return ET.tostring(root, encoding='utf-8', xml_declaration=True)

    def save(self, output):
        manifest = ET.Element(q('manifest:manifest'), {q('manifest:version'):'1.2'})
        for path, mime in [('/', 'application/vnd.oasis.opendocument.text'), ('content.xml','text/xml'),
                           ('styles.xml','text/xml'), ('meta.xml','text/xml')]:
            add(manifest, 'manifest:file-entry', {'manifest:full-path':path, 'manifest:media-type':mime})
        meta = ET.Element(q('office:document-meta'), {q('office:version'):'1.2'})
        add(add(meta, 'office:meta'), 'dc:title', text='UWB·IMU 드론의 위치추정과 학습 제어')
        styles_data = self.styles_xml()
        with ZipFile(output, 'w', ZIP_DEFLATED) as z:
            z.writestr('mimetype', 'application/vnd.oasis.opendocument.text', compress_type=ZIP_STORED)
            z.writestr('content.xml', ET.tostring(self.root, encoding='utf-8', xml_declaration=True))
            z.writestr('styles.xml', styles_data)
            z.writestr('meta.xml', ET.tostring(meta, encoding='utf-8', xml_declaration=True))
            z.writestr('META-INF/manifest.xml', ET.tostring(manifest, encoding='utf-8', xml_declaration=True))


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser()
    parser.add_argument('markdown', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    content = args.markdown.read_text(encoding='utf-8')
    refs = {int(n):v for n,v in re.findall(r'^\[\^(\d+)\]: (.+)$', content, re.M)}
    body = re.sub(r'^\[\^\d+\]: .+$', '', content, flags=re.M)
    doc = Document(refs)
    doc.markdown(body)
    doc.sources()
    doc.save(args.output)
    print(f'Created {args.output}; references={len(refs)}, cited={len(doc.cited)}')
    if set(refs)-doc.cited:
        print('Uncited reference numbers:', sorted(set(refs)-doc.cited))


if __name__ == '__main__':
    main()
