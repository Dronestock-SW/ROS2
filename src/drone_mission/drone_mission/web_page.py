"""Local operator page. Saving a draft never dispatches a flight request."""

PAGE = '''<!doctype html><html lang="ko"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Dronestock 비행 시험 · 미션 관리</title>
<style>body{max-width:960px;margin:30px auto;padding:0 18px 200px;font:16px sans-serif;background:#f6f7f9;color:#182332}
section{padding:18px;background:white;border-radius:10px;margin:16px 0}h2{font-size:19px;margin-top:0}
input,select{padding:10px;max-width:100%;box-sizing:border-box}textarea{box-sizing:border-box;width:100%;min-height:100px;font:14px monospace;padding:12px}
button{padding:11px 15px;margin:7px 6px 7px 0;border:0;border-radius:6px;background:#164abd;color:white;cursor:pointer}
button:disabled{background:#8d99aa;cursor:not-allowed}.quiet{background:#536477}#land{background:#ae3030}#land:disabled{background:#8d99aa}
table{width:100%;border-collapse:collapse;font-size:14px}td,th{text-align:left;padding:9px;border-bottom:1px solid #ddd}
pre{white-space:pre-wrap;overflow-wrap:anywhere;font-size:13px}.hint{color:#526176;font-size:14px}#notice{padding:12px;background:#e8eef9;white-space:pre-wrap}
.flightbar{position:fixed;bottom:0;left:0;right:0;z-index:10;background:#fff;border-top:2px solid #d9e1ed;box-shadow:0 -3px 14px #18233218;padding:10px 18px calc(10px + env(safe-area-inset-bottom));box-sizing:border-box}.flightbar>div{max-width:960px;margin:auto}.flightbar p{margin:3px 0;font-size:14px}.flightbar button{margin:5px 4px 3px 0}#readiness{color:#7d3a12}#checks{margin-top:12px}
@media(max-width:550px){body{padding-bottom:245px}.flightbar button{font-size:12px;padding:10px 8px}}
</style><h1>Dronestock 비행 시험</h1>
<p>미션을 저장하고 선택한 뒤 START로 실행합니다. 저장·불러오기·삭제는 기체를 움직이지 않습니다.</p>
<p><a href="http://127.0.0.1:8350/">제자리 이륙·2초 호버·착륙 전용 시험</a> · 전체 창고 미션 준비와 별도입니다.</p><section><h2>현장 설정</h2><label for="ceiling">천장 높이 · 바닥 기준 m</label>
<input id="ceiling" type="number" min="0.5" max="100" step="0.1" placeholder="예: 3.0">
<button id="setceiling">천장 설정 저장</button><span id="site"></span>
<p class="hint">천장은 실측 지도의 상한을 제한합니다. 이륙 높이는 PX4 설정을 사용합니다. 출발점은 START 시 검증된 PX4 기체 위치에서 자동 확인합니다.</p></section>
<section><h2>저장 미션</h2><label for="saved">미션 목록 </label><select id="saved"><option value="">저장 미션 없음</option></select>
<button id="load">불러오기</button><button id="new" class="quiet">새 미션</button><button id="delete" class="quiet">선택 미션 삭제</button>
<p id="selection" class="hint">새 미션 · 아직 저장하지 않았습니다.</p>
<label for="name">미션 이름 </label><input id="name" maxlength="80" placeholder="예: 1층 호버 시험">
<label for="preset">시험 종류 </label><select id="preset"><option value="hover">이륙·2초 호버·착륙</option><option value="x">X 1m 왕복</option><option value="y">Y 1m 왕복</option><option value="xy">X/Y 각 1m·역순 복귀</option><option value="custom">직접 경유지 입력</option></select>
<button id="save">미션 저장</button><button id="make" class="quiet">시험 경로 미리보기</button>
<details><summary>고급 · A1 기준 수평 경유지 JSON</summary><label for="route">x/y 단위 m · z 목표는 입력하지 않습니다.</label><textarea id="route">[]</textarea></details>
<p class="hint">시험 종류는 앵커 연결 전에도 저장할 수 있습니다. 실제 경로는 START 때 출발점에서 다시 계산합니다. 같은 내용의 중복 저장은 기존 미션을 선택합니다.</p></section>
<section><h2>비행 준비 확인</h2><p>저장 미션 불러오기 → 준비 확인 → 하단 START → ARM·이륙 → 미션 → 착륙. 별도로 먼저 시동을 걸지 않습니다.</p>
<p id="notice" role="status">저장 미션을 선택하거나 새로 작성하세요.</p><div id="checks"></div></section>
<section><h2>실행 이력</h2><p class="hint">삭제한 저장 미션의 실행 이력도 보존됩니다. 기록 닫기는 비행 취소나 RC 인계 해제가 아닙니다. 다음 비행은 실행기가 IDLE인 새 지상 세션에서 시작합니다.</p>
<table><thead><tr><th>실행</th><th>미션</th><th>상태·결과</th><th>관리</th></tr></thead><tbody id="history"></tbody></table></section>
<details><summary>실시간 관측 상세</summary><pre id="state">상태 대기</pre></details>
<footer class="flightbar" aria-label="비행 명령"><div><p id="runstate">연결 확인 중</p><p id="readiness" role="status">연결 확인 전 · START 대기</p>
<button id="start" disabled>미션 START · ARM·이륙 포함</button><button id="home" disabled>출발점 복귀 · 착륙</button><button id="land" disabled>착륙 요청</button><p class="hint">회색 START는 준비 조건 미충족입니다. 체크는 수신 상태와 검증 근거로 갱신됩니다.</p></div></footer>
<script>
const by=id=>document.getElementById(id),notice=by('notice'),route=by('route');
let catalog={drafts:[],runs:[]},selected=null,dirty=false,busy=false,lastStatus={},pending=null;
function definition(){return by('preset').value==='custom'?{route_tasks:JSON.parse(route.value)}:{trial_case:by('preset').value};}
function markDirty(){dirty=true;by('selection').textContent=selected?'수정 중 · 저장해야 선택 미션에 반영됩니다.':'새 미션 · 아직 저장하지 않았습니다.';}
by('name').oninput=markDirty;by('preset').onchange=markDirty;route.oninput=()=>{by('preset').value='custom';markDirty();};
async function post(path,body){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});const d=await r.json();if(!r.ok){const e=Error(d.error);e.responded=true;throw e;}return d;}
function loadDraft(id){const d=catalog.drafts.find(x=>x.id===id);if(!d)return;selected={id:d.id,revision:d.revision};dirty=false;
by('name').value=d.name;by('preset').value=d.definition.trial_case??'custom';route.value=JSON.stringify(d.definition.route_tasks??[],null,2);by('saved').value=d.id;by('selection').textContent=d.name+' · 버전 '+d.revision+' · 저장됨';}
async function refreshCatalog(){const r=await fetch('/local/missions',{cache:'no-store'});if(!r.ok)throw Error('미션 목록 수신 실패');catalog=await r.json();
const choice=by('saved').value;by('saved').replaceChildren(new Option('미션 선택', ''));for(const d of catalog.drafts)by('saved').add(new Option(d.name+' · v'+d.revision,d.id));by('saved').value=choice;
by('history').replaceChildren();for(const run of catalog.runs){const tr=document.createElement('tr');
for(const text of [String(run.id),run.name,run.state+' · '+(run.result.work_outcome??'결과 대기')+(run.open?' · 기록 열림':' · 기록 닫힘')]){const td=document.createElement('td');td.textContent=text;tr.appendChild(td);}
const td=document.createElement('td'),button=document.createElement('button');button.textContent='기록 닫기';button.disabled=!run.open;button.className='quiet';button.onclick=async()=>{try{await post('/local/missions',{action:'close_run',run_id:run.id});notice.textContent='기록을 닫았습니다. 기체 실행 상태는 초기화하지 않습니다.';await refreshCatalog();}catch(e){notice.textContent=e.message;}};td.appendChild(button);tr.appendChild(td);by('history').appendChild(tr);}}
by('load').onclick=()=>{loadDraft(by('saved').value);notice.textContent='저장 미션을 불러왔습니다. 비행 명령은 보내지 않았습니다.';};
by('new').onclick=()=>{selected=null;dirty=false;by('name').value='';by('preset').value='hover';route.value='[]';by('saved').value='';by('selection').textContent='새 미션 · 아직 저장하지 않았습니다.';};
by('save').onclick=async()=>{try{const d=await post('/local/missions',{action:'save',name:by('name').value,definition:definition(),draft_id:selected?.id,revision:selected?.revision});await refreshCatalog();loadDraft(d.draft_id);notice.textContent=d.duplicate?'같은 내용의 기존 미션을 선택했습니다.':'미션 저장 완료 · 비행 명령 없음';}catch(e){notice.textContent=e.message;}};
by('delete').onclick=async()=>{try{const d=catalog.drafts.find(x=>x.id===by('saved').value);if(!d)throw Error('삭제할 미션을 선택하세요.');await post('/local/missions',{action:'archive',draft_id:d.id,revision:d.revision});if(selected?.id===d.id)by('new').click();await refreshCatalog();notice.textContent='저장 미션 삭제 완료 · 실행 이력은 보존했습니다.';}catch(e){notice.textContent=e.message;}};
by('setceiling').onclick=async()=>{try{await post('/local/command',{action:'set_ceiling',ceiling_height_m:Number(by('ceiling').value)});notice.textContent='천장 설정 저장 완료';}catch(e){notice.textContent=e.message;}};
async function command(action){if(busy)return;busy=true;by('start').disabled=true;
try{let body;if(pending&&pending.action===action){body=pending;}
else{if(action==='start'&&pending)throw Error('응답 미확인 요청의 수락 여부를 먼저 확인하세요.');body={action,client_request_id:crypto.randomUUID(),expected_drone_id:lastStatus.assignment.drone_id,expected_layout_id:lastStatus.assignment.anchor_layout_id};if(action==='start'){if(selected&&dirty)throw Error('변경한 미션을 먼저 저장하세요.');if(selected){body.draft_id=selected.id;body.revision=selected.revision;}else Object.assign(body,definition());}pending=body;sessionStorage.setItem('pendingMissionCommand',JSON.stringify(body));}
const d=await post('/local/command',body);pending=null;sessionStorage.removeItem('pendingMissionCommand');notice.textContent='요청 접수 · 실행 '+d.mission_db_id+' · 기체 수락·진행 상태를 확인하세요.';await refreshCatalog();
}catch(e){if(e.responded){pending=null;sessionStorage.removeItem('pendingMissionCommand');}notice.textContent=e.message+(pending?' · 응답 미확인: 같은 버튼으로 확인하세요.':'');}finally{busy=false;}}
try{pending=JSON.parse(sessionStorage.getItem('pendingMissionCommand'));}catch(e){sessionStorage.removeItem('pendingMissionCommand');}
by('start').onclick=()=>command('start');by('home').onclick=()=>command('return_to_home');by('land').onclick=()=>command('land');
by('make').onclick=async()=>{try{if(by('preset').value==='custom')throw Error('직접 경유지는 JSON 내용을 확인하세요.');const r=await fetch('/local/preset?'+new URLSearchParams({case:by('preset').value})),d=await r.json();if(!r.ok)throw Error(d.error);route.value=JSON.stringify(d.route_tasks,null,2);notice.textContent='현재 출발점 '+JSON.stringify(d.start_xy_m)+' · START 때 다시 계산합니다.';}catch(e){notice.textContent=e.message;}};
let polls=0;
async function update(){try{const r=await fetch('/local/status',{cache:'no-store'}),s=await r.json(),t=s.telemetry;lastStatus=s;
by('start').disabled=busy||(!s.can_start&&pending?.action!=='start');
const control=s.telemetry_fresh&&t.fc_connected&&s.active_run_id!=null&&['mission_db_id','mission_code','route_revision'].every(k=>t[k]===s.assignment[k])&&!['IDLE','END','LANDED','PILOT_OVERRIDE','FAILED','UNCONFIRMED'].includes(t.flight_state);
by('land').disabled=busy||(!control&&pending?.action!=='land');by('home').disabled=busy||(!control&&pending?.action!=='return_to_home');
by('setceiling').disabled=!s.site_editable;by('site').textContent=(s.site.ceiling_height_m??'미설정')+' m · 이륙 '+s.expected_takeoff_height_m+' m';
by('runstate').textContent=(s.active_run_id==null?'진행 미션 없음':'실행 '+s.active_run_id+' · 중복 START 잠금')+' / '+(t.flight_state??'연결 대기')+' / '+(t.fc_mode??'모드 미확인');
const checks=(t.preflight?.checks??[]).map(c=>({...c,title:c.code==='estimator'?'PX4 수평 위치 추정 유효':c.title})),remaining=checks.filter(c=>!c.passed);by('readiness').textContent=!s.telemetry_fresh?'기체 연결 확인 대기':s.active_run_id!=null?'진행 미션 확인 중 · 새 START 잠금':s.can_start?'시작 조건 충족 · 선택 미션과 주변을 확인 후 START':remaining.length?'준비 대기 '+remaining.length+'개 · '+remaining.slice(0,3).map(c=>c.title).join(' / '):'START 대기 · 실행기·지상 상태 확인';
by('checks').replaceChildren();for(const c of checks){const row=document.createElement('div');row.textContent=(c.passed?'✓ ':'대기 · ')+c.title;by('checks').appendChild(row);}
by('state').textContent=JSON.stringify({기체:s.assignment.drone_id,배치:s.assignment.anchor_layout_id,텔레메트리수신:s.telemetry_fresh,상태:t.flight_state,사유:t.flight_reason,시동:t.fc_armed,PX4모드:t.fc_mode,UWB_XYZ:[t.x,t.y,t.current_z_m],Z출처:t.current_z_source,Z지상기준추정:t.current_z_estimated,Z실측유효:t.current_z_trusted,XYZ실측유효:t.xyz_valid,PX4_ENU:t.px4_position_enu_m,자동출발점_창고XY:s.automatic_start_xy_m,고정복귀점_창고XY:t.home_xy_m,착륙확인:t.landing_verified,출발점복귀확인:t.home_verified,비행결과:t.flight_outcome,작업결과:t.work_outcome,스캔결과:t.scan_results,요청검사:t.target_validation},null,2);
if(++polls%6===0)await refreshCatalog();
}catch(e){by('start').disabled=true;by('land').disabled=true;by('home').disabled=true;by('runstate').textContent='연결 대기 · 이전 미션을 자동 실행하지 않습니다.';by('readiness').textContent='웹 연결 끊김 · START 대기';}setTimeout(update,500);}
refreshCatalog().catch(e=>{notice.textContent=e.message;});update();
</script></html>'''.encode('utf-8')
