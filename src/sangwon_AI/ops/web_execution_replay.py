"""Render immutable web/C++/synthetic-FC evidence as an interactive replay.

The generated page has no command endpoint or FC connection. Source hashes
and the execution backend stay visible; replay is never labelled live flight.
"""
import argparse
import hashlib
import json
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--evidence',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();cases={}
    labels={'complete':'정상 이륙·호버·착륙','lidar_sdk_gap':'SDK LiDAR + UWB 0.6초 공백',
            'lidar_loss':'LiDAR·UWB 손실','cpp_loss':'C++ 판단 프로세스 중단','rc_takeover':'조종자 인계'}
    native=(args.evidence/'nominal/evidence.json').is_file()
    if native:
        labels={'nominal':'C++ 실행기 정상 완료','python_pause':'Python 1초 정지 중 C++ 실행',
                'lidar_gap':'C++ 실행기 + SDK LiDAR 공백 보완','lidar_loss':'UWB·LiDAR 손실 보호','pilot':'조종자 인계'}
    for key,label in labels.items():
        source=args.evidence/key/'evidence.json' if native else args.evidence/('executor-'+key+'.json')
        raw=source.read_bytes();data=json.loads(raw)
        assert data['physical_flight'] is False
        if native:
            end=data['state'];data['state']=end['state'];data['reason']=end['reason']
            data['execution_backend']='sangwon_cpp_executor'
            data['ai']=dict(binary_sha256=data.get('binary_sha256','미기록'))
            for row in data['trace']:
                decision=row['cpp_decision']
                row['ai']=dict(decision_age_s=row.get('decision_age_s'),last_roundtrip_s=None)
                row['horizontal_observation']=dict(decision.get('lidar_support',{}),gap_supported=decision.get('gap_supported',False))
        cases[key]=dict(label=label,sha256=hashlib.sha256(raw).hexdigest(),data=data)
    payload=json.dumps(cases,ensure_ascii=False,allow_nan=False).replace('<','\\u003c')
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(PAGE.replace('__DATA__',payload),encoding='utf-8')
    print(str(args.output.resolve()))


PAGE='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dronestock | 웹 미션 검증 예시</title><style>
*{box-sizing:border-box}body{margin:0;background:#0d1420;color:#e7effb;font:15px system-ui,"Malgun Gothic",sans-serif}
main{max-width:1250px;margin:auto;padding:30px}header{display:flex;justify-content:space-between;gap:20px;align-items:center}
h1{font-size:26px;margin:0 0 7px}p{color:#aebfd6;line-height:1.6;margin:7px 0}.pill{border:1px solid #40546f;padding:8px 12px;border-radius:22px;color:#b9d4f7}
.notice{background:#263021;border:1px solid #647349;color:#e7edba;padding:14px 18px;border-radius:12px;margin:20px 0}
.controls{display:flex;gap:12px;flex-wrap:wrap;margin:20px 0}button,select{font:inherit;border:1px solid #3b506a;background:#172334;color:#e7effb;padding:10px 15px;border-radius:9px}
button{cursor:pointer}button.primary{background:#39b8a2;border-color:#39b8a2;color:#09201e;font-weight:700}
.grid{display:grid;grid-template-columns:2fr 1fr;gap:18px}.card{background:#151f2e;border:1px solid #2a3b53;border-radius:15px;padding:20px}
.metrics{display:grid;grid-template-columns:repeat(4,1fr);gap:12px;margin-bottom:18px}.value{font-size:25px;font-weight:650;margin-top:7px}.label{color:#9fafc5;font-size:13px}
.flow{display:flex;gap:6px;flex-wrap:wrap;margin:14px 0}.step{padding:9px 12px;background:#1d2c40;border-radius:7px;color:#8799b2}.step.active{background:#276c64;color:#dcfff5}.step.done{color:#b9decf}
svg{width:100%;display:block}h2{font-size:16px;margin:0 0 14px}.line{display:flex;justify-content:space-between;padding:11px 0;border-bottom:1px solid #29384e;gap:12px}.good{color:#65d7ba}.warn{color:#f5cb6d}.mono{font:12px ui-monospace,monospace;word-break:break-all;color:#a7bfd8}
@media(max-width:800px){.grid{grid-template-columns:1fr}.metrics{grid-template-columns:repeat(2,1fr)}main{padding:18px}header{display:block}}
</style><main><header><div><h1>웹 미션 검증 예시</h1><p>웹 START → C++ AI 판단 → 실행기 → 모사 FC → 완료 확인</p></div><span class="pill">원본 검증기록 재생</span></header>
<div class="notice"><strong>실제 드론과 연결되지 않은 예시입니다.</strong> 버튼은 저장된 기록만 재생합니다. 실제 START·ARM·이륙 명령을 보내지 않습니다.</div>
<div class="controls"><select id="case" aria-label="검증 시나리오"></select><button id="play" class="primary">시작 기록 재생</button><button id="pause">일시정지</button><button id="reset">처음으로</button><select id="speed" aria-label="재생 속도"><option value="1">1배속</option><option value="2" selected>2배속</option><option value="4">4배속</option></select></div>
<div class="metrics"><div class="card"><div class="label">미션 단계</div><div id="phase" class="value">준비</div></div><div class="card"><div class="label">출발면 대비 높이 · 모사 FC</div><div id="height" class="value">0.00 m</div></div><div class="card"><div class="label">고정 출발 XY 이탈 · 모사 FC</div><div id="drift" class="value">0.000 m</div></div><div class="card"><div class="label">원본 기록 경과 시간</div><div id="time" class="value">0.0 s</div></div></div>
<div class="grid"><section class="card"><h2>수직 이륙 → 연속 호버 10초 → 복귀 확인 → 착륙</h2><div id="flow" class="flow"></div><svg id="chart" viewBox="0 0 760 300" role="img" aria-label="모사 FC 높이 기록"></svg><p>목표 높이 1.7m · 상승 제한 0.5m/s · XY 경로 길이 0m</p><div class="line"><span>원본 최종 판정</span><strong id="outcome"></strong></div><div class="line"><span>현재 재생 지점의 UWB 나이</span><span id="uwb"></span></div><div class="line"><span>LiDAR 보완 상태</span><span id="lidar"></span></div></section>
<aside class="card"><h2>연결·검증 범위</h2><div class="line"><span>AI 판단</span><strong>C++ BehaviorTree</strong></div><div class="line"><span>FC 실행기 · 이 기록</span><strong id="executor">Python</strong></div><div class="line"><span>차량</span><strong class="warn">모사 FC</strong></div><div class="line"><span>실비행</span><strong class="warn">미실시</strong></div><div class="line"><span>C++ 결정 나이</span><span id="age"></span></div><div class="line"><span>C++ 왕복 시간</span><span id="roundtrip"></span></div><div class="line"><span>확인한 명령 순서</span><span id="commands"></span></div><p>현재 실기 준비는 별도입니다. UWB 기준점·장착 yaw·이동 대조·호스트 주기 안정성 확인이 남아 있습니다.</p><p class="label">원본 검증 파일 SHA-256</p><div class="mono" id="source"></div><p class="label">실제 C++ 실행 파일 SHA-256</p><div class="mono" id="binary"></div><p class="label">판단 상태·종료 사유</p><div class="mono" id="reason"></div></aside></div></main>
<script>
const evidence=__DATA__,phases=['PREPARING','ARMING','TAKING_OFF','MOVING','DWELL','RETURNING','LANDING','END'];
const names={PREPARING:'출발 준비',ARMING:'시동 확인',TAKING_OFF:'수직 이륙',MOVING:'제자리 안정화',DWELL:'연속 호버',RETURNING:'복귀 확인',LANDING:'착륙 확인',END:'미션 완료',FAILED:'실패 확인',UNCONFIRMED:'미확인 중단',PILOT_OVERRIDE:'조종자 인계',GROUND_RECOVERY:'지상 복구'};
const $=id=>document.getElementById(id);for(const [key,c] of Object.entries(evidence)){const o=document.createElement('option');o.value=key;o.textContent=c.label;$('case').append(o)}
let elapsed=0,playing=false,last=performance.now();function current(){return evidence[$('case').value]}function reset(){elapsed=0;playing=false;last=performance.now();render()}
function render(){const c=current(),rows=c.data.trace,base=rows[0].height_m,home=rows[0].xy;const row=[...rows].reverse().find(x=>x.elapsed_s<=elapsed)||rows[0],i=phases.indexOf(row.state);$('phase').textContent=names[row.state]||row.state;$('height').textContent=(row.height_m-base).toFixed(2)+' m';$('drift').textContent=Math.hypot(row.xy[0]-home[0],row.xy[1]-home[1]).toFixed(3)+' m';$('time').textContent=elapsed.toFixed(1)+' s';
$('flow').innerHTML=phases.map((p,j)=>'<span class="step '+(j===i?'active':j<i?'done':'')+'">'+names[p]+'</span>').join('');$('outcome').textContent=(names[c.data.state]||c.data.state)+' · '+c.data.state;$('outcome').className=c.data.state==='END'?'good':'warn';$('uwb').textContent=(row.uwb_age_s||0).toFixed(3)+' s';const l=row.horizontal_observation||{};$('lidar').textContent=l.enabled===false?'보완 비활성':l.gap_supported?'UWB 공백 중 기존 목표 유지':l.approved?'정합·기준점 검사 통과':l.reason||'미사용';$('lidar').className=l.gap_supported?'good':'';
$('executor').textContent=c.data.execution_backend==='sangwon_cpp_executor'?'C++':'Python';$('age').textContent=['END','FAILED','PILOT_OVERRIDE','UNCONFIRMED'].includes(row.state)?'종료 · 목표 송신 중지':row.ai.decision_age_s==null?'미기록':(row.ai.decision_age_s*1000).toFixed(1)+' ms';$('roundtrip').textContent=c.data.execution_backend==='sangwon_cpp_executor'?'같은 C++ 프로세스':row.ai.last_roundtrip_s==null?'미기록':(row.ai.last_roundtrip_s*1000).toFixed(1)+' ms';$('commands').textContent=c.data.commands.join(' → ');$('source').textContent=c.sha256;$('binary').textContent=c.data.ai.binary_sha256;$('reason').textContent=c.data.reason;
const end=rows.at(-1).elapsed_s,X=t=>45+t/end*690,Y=h=>260-h/2*215;let html='';for(const h of [0,.5,1,1.5,2])html+='<path d="M45 '+Y(h)+'H735" stroke="#2d4059"/><text x="5" y="'+(Y(h)+4)+'" fill="#97abc5" font-size="12">'+h+'m</text>';html+='<path d="M45 '+Y(1.7)+'H735" stroke="#73906b" stroke-dasharray="5 5"/><text x="590" y="'+(Y(1.7)-9)+'" fill="#bdd7ac" font-size="12">목표 1.7m</text>';const seen=rows.filter(x=>x.elapsed_s<=elapsed);if(!seen.length)seen.push(rows[0]);html+='<polyline points="'+seen.map(r=>X(r.elapsed_s)+','+Y(r.height_m-base)).join(' ')+'" fill="none" stroke="#54d1ba" stroke-width="3"/><circle cx="'+X(Math.min(elapsed,end))+'" cy="'+Y(row.height_m-base)+'" r="5" fill="#fff"/>';for(const t of [0,end/2,end])html+='<text x="'+X(t)+'" y="290" fill="#97abc5" font-size="12">'+t.toFixed(1)+'s</text>';$('chart').innerHTML=html;}
$('play').onclick=()=>{if(elapsed>=current().data.trace.at(-1).elapsed_s)elapsed=0;playing=true;last=performance.now()};$('pause').onclick=()=>playing=false;$('reset').onclick=reset;$('case').onchange=reset;
function loop(t){if(playing){elapsed=Math.min(current().data.trace.at(-1).elapsed_s,elapsed+(t-last)/1000*Number($('speed').value));if(elapsed>=current().data.trace.at(-1).elapsed_s)playing=false;render()}last=t;requestAnimationFrame(loop)}reset();requestAnimationFrame(loop);
</script></html>'''


if __name__=='__main__':main()
