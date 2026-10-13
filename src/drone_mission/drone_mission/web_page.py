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
<section><h2>현장 설정</h2><label for="ceiling">천장 높이 · 바닥 기준 m</label>
<input id="ceiling" type="number" min="0.5" max="100" step="0.1" placeholder="예: 3.0">
<button id="setceiling">천장 설정 저장</button><span id="site"></span>
<p class="hint">이륙 높이는 PX4 설정을 사용합니다. 출발점은 START 시 PX4 기체 위치를 앵커 좌표로 변환해 확인합니다.</p><p id="profile" class="hint"></p></section>
<section><h2>UWB 관측 연결</h2><p id="uwbpolicy" role="status">기체의 적용 상태 확인 대기</p><p class="hint">현재 버전은 XY 점프에 따른 관측 차단을 사용하지 않습니다. 새 관측의 시각·유효성과 거리 기하 검사는 유지합니다.</p></section>
<section><h2>저장 미션</h2><label for="saved">미션 목록 </label><select id="saved"><option value="">저장 미션 없음</option></select>
<button id="load">불러오기</button><button id="new" class="quiet">새 미션</button><button id="delete" class="quiet">선택 미션 삭제</button>
<p id="selection" class="hint">새 미션 · 아직 저장하지 않았습니다.</p>
<label for="name">미션 이름 </label><input id="name" maxlength="80" placeholder="예: 1층 호버 시험">
<label for="preset">시험 종류 </label><select id="preset"><option value="hover">이륙·10초 호버·착륙</option><option value="x">+X 1m·복귀·착륙</option><option value="y">+Y 1m·복귀·착륙</option><option value="xy">+X→+Y 각 1m·역순 복귀·착륙</option><option value="custom">직접 경유지 입력</option></select>
<button id="save">미션 저장</button><button id="make" class="quiet">시험 경로 미리보기</button>
<details><summary>고급 · A1 기준 수평 경유지 JSON</summary><label for="route">x/y 단위 m · z 목표는 입력하지 않습니다.</label><textarea id="route">[]</textarea></details>
<p class="hint">시험 종류는 앵커 연결 전에도 저장할 수 있습니다. 실제 경로는 START 때 출발점에서 다시 계산합니다. 같은 내용의 중복 저장은 기존 미션을 선택합니다.</p></section>
<section><h2>비행 준비 확인</h2><p id="flow">저장 미션 불러오기 → 준비 확인 → 하단 START → ARM·이륙 → 미션 → 착륙. 별도로 먼저 시동을 걸지 않습니다.</p>
<p id="notice" role="status">저장 미션을 선택하거나 새로 작성하세요.</p><div id="checks"></div><details><summary>참고 관측 · START 조건에서 제외</summary><div id="observations"></div></details></section>
<section><h2>실행 이력</h2><p class="hint">삭제한 저장 미션의 실행 이력도 보존됩니다. 기록 닫기는 비행 취소나 RC 인계 해제가 아닙니다. 다음 비행은 실행기가 IDLE인 새 지상 세션에서 시작합니다.</p>
<table><thead><tr><th>실행</th><th>미션</th><th>상태·결과</th><th>관리</th></tr></thead><tbody id="history"></tbody></table></section>
<details><summary>실시간 관측 상세</summary><pre id="state">상태 대기</pre></details>
<footer class="flightbar" aria-label="비행 명령"><div><p id="runstate">연결 확인 중</p><p id="readiness" role="status">연결 확인 전 · START 대기</p>
<button id="takeoff" hidden disabled>이륙 · PX4 TAKEOFF → HOLD</button><button id="start" disabled>미션 START · ARM·이륙 포함</button><button id="arm" disabled>ARM · 수동 시동만</button><button id="home" disabled>출발점 복귀 · 착륙</button><button id="land" disabled>착륙 요청</button><p id="commandhint" class="hint">ARM은 현재 수동 모드에서 시동만 요청합니다. 미션 START는 DISARM 상태에서 사용합니다. 요청 수락과 실제 시동 상태를 구분해 확인하세요.</p><p id="armstate" role="status"></p></div></footer>
<script>
const by=id=>document.getElementById(id),notice=by('notice'),route=by('route');
let catalog={drafts:[],runs:[]},selected=null,dirty=false,busy=false,landBusy=false,commandGeneration=0,lastStatus={},pending=null;
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
async function command(action){const urgent=['land','takeoff_land'].includes(action);if(urgent?landBusy:(busy||landBusy))return;const generation=++commandGeneration;if(urgent)landBusy=true;else busy=true;by('start').disabled=true;by('arm').disabled=true;
try{let body;if(pending&&pending.action===action){body=pending;}
else{if(action==='start'&&pending)throw Error('응답 미확인 요청의 수락 여부를 먼저 확인하세요.');body={action,client_request_id:crypto.randomUUID(),expected_drone_id:lastStatus.assignment.drone_id,expected_layout_id:lastStatus.assignment.anchor_layout_id};if(action==='start'){if(selected&&dirty)throw Error('변경한 미션을 먼저 저장하세요.');if(selected){body.draft_id=selected.id;body.revision=selected.revision;}else Object.assign(body,definition());}pending=body;sessionStorage.setItem('pendingMissionCommand',JSON.stringify(body));}
if(['arm','takeoff','takeoff_land'].includes(action)&&!body.operator_session_id){body.operator_session_id=lastStatus.telemetry.operator_session_id;sessionStorage.setItem('pendingMissionCommand',JSON.stringify(body));}
const d=await post('/local/command',body);if(generation!==commandGeneration)return;pending=null;sessionStorage.removeItem('pendingMissionCommand');notice.textContent=action==='takeoff'?'이륙 요청 접수 · HOLD 확인 후 별도 START를 누르세요.':action==='takeoff_land'?'착륙 요청 접수 · 실제 착륙을 확인하세요.':action==='arm'?'ARM 요청 접수 · PX4 응답과 실제 시동 상태를 확인하세요.':'요청 접수 · 실행 '+d.mission_db_id+' · 기체 수락·진행 상태를 확인하세요.';await refreshCatalog();
}catch(e){if(generation!==commandGeneration)return;if(e.responded){pending=null;sessionStorage.removeItem('pendingMissionCommand');}notice.textContent=e.message+(pending?' · 응답 미확인: 같은 버튼으로 확인하세요.':'');}finally{if(urgent)landBusy=false;else busy=false;}}
try{pending=JSON.parse(sessionStorage.getItem('pendingMissionCommand'));}catch(e){sessionStorage.removeItem('pendingMissionCommand');}
by('start').onclick=()=>command('start');by('home').onclick=()=>command('return_to_home');by('land').onclick=()=>command(lastStatus.active_run_id==null&&lastStatus.start_mode==='takeoff_hold'?'takeoff_land':'land');
by('takeoff').onclick=()=>command('takeoff');
by('arm').onclick=()=>command('arm');
by('make').onclick=async()=>{try{if(by('preset').value==='custom')throw Error('직접 경유지는 JSON 내용을 확인하세요.');const r=await fetch('/local/preset?'+new URLSearchParams({case:by('preset').value})),d=await r.json();if(!r.ok)throw Error(d.error);route.value=JSON.stringify(d.route_tasks,null,2);notice.textContent='현재 출발점 '+JSON.stringify(d.start_xy_m)+' · START 때 다시 계산합니다.';}catch(e){notice.textContent=e.message;}};
let polls=0;
async function update(){try{const r=await fetch('/local/status',{cache:'no-store'}),s=await r.json(),t=s.telemetry;lastStatus=s;
const up=t.uwb_pipeline??{};by('uwbpolicy').textContent=[['btf','B_TF 입력'],['sender','PX4 송신']].map(([key,label])=>{const p=up[key];return label+': '+(!s.telemetry_fresh||!p?.fresh?'확인 대기':p.jump_guard_enabled===false?'점프 차단 제거 확인':p.jump_guard_enabled===true?'이전 점프 차단 사용 중':'적용 여부 미확인')+(p?.fresh&&p.reason?' · '+p.reason:'');}).join(' / ');
by('takeoff').hidden=s.start_mode!=='takeoff_hold';by('takeoff').disabled=busy||(!s.can_takeoff&&pending?.action!=='takeoff');
by('arm').hidden=s.start_mode==='takeoff_hold';
if(s.start_mode==='takeoff_hold'){by('start').textContent='미션 START · HOLD에서 시작';by('flow').textContent='웹 이륙 → PX4 TAKEOFF → HOLD 안정 확인 → 별도 START → 미션 → 착륙';by('commandhint').textContent='이륙은 PX4가 수행합니다. HOLD에서는 START를 누를 때까지 미션이 시작되지 않습니다.';by('preset').options[0].textContent='10초 호버·착륙';}
if(s.takeoff_backend==='local_position'){by('takeoff').textContent='웹 자동 이륙 · 현재 XY 유지';by('start').textContent='미션 START · POSITION에서 시작';by('flow').textContent='웹 자동 이륙 → POSITION 대기 → 별도 START → OFFBOARD 재진입 → 미션 → 착륙';by('commandhint').textContent='현재 XY에서 자동 상승합니다. POSITION 대기 중에는 RC 스틱을 중립에 두세요. 미션은 별도 START로 시작합니다.';}
by('start').disabled=busy||(!s.can_start&&pending?.action!=='start');
by('arm').disabled=busy||(!s.can_arm&&pending?.action!=='arm');
const ar=t.operator_arm_result;by('armstate').textContent=(t.fc_armed===true?'실제 시동: ARM':t.fc_armed===false?'실제 시동: DISARM':'실제 시동: 미확인')+(ar?' · ARM 요청: '+ar.reason:'');
const control=s.telemetry_fresh&&t.fc_connected&&s.active_run_id!=null&&['mission_db_id','mission_code','route_revision'].every(k=>t[k]===s.assignment[k])&&!['IDLE','END','LANDED','PILOT_OVERRIDE','FAILED','UNCONFIRMED'].includes(t.flight_state);
by('land').disabled=landBusy||(!control&&!(s.telemetry_fresh&&t.native_takeoff?.can_land)&&!['land','takeoff_land'].includes(pending?.action));by('home').disabled=busy||(!control&&pending?.action!=='return_to_home');
by('setceiling').disabled=!s.site_editable;by('site').textContent=(s.site.ceiling_height_m??'미설정')+' m · 이륙 '+s.expected_takeoff_height_m+' m';
by('profile').textContent=(s.command_frame==='local_xy'?'A1 기준 로컬 XY · 위도·경도 사용 안 함 · ':'')+(s.execution_profile==='anchor_trial'?'앵커 좌표 이동 시험 · 장애물 지도 검사 없음':'실측 지도 미션')+' · 수평 이동 요청 속도 '+s.speed_m_s+' m/s · 방위는 PX4가 유지합니다.';
by('runstate').textContent=(s.active_run_id==null?'진행 미션 없음':'실행 '+s.active_run_id+' · 중복 START 잠금')+' / '+(t.flight_state??'연결 대기')+' / '+(t.fc_mode??'모드 미확인')+(t.mission_ai?.backend==='sangwon_cpp_bt'?' / C++ AI '+(t.mission_ai.connected?(t.mission_ai.phase??'연결됨'):'연결 대기'):'');
const checks=(t.preflight?.checks??[]).map(c=>({...c,title:c.code==='estimator'?'PX4 수평 위치 추정 유효':c.title})),remaining=checks.filter(c=>c.required!==false&&!c.passed);by('readiness').textContent=!s.telemetry_fresh?'기체 연결 확인 대기':s.active_run_id!=null?'진행 미션 확인 중 · 새 START 잠금':s.can_start?'시작 조건 충족 · 선택 미션과 주변을 확인 후 START':remaining.length?'준비 대기 '+remaining.length+'개 · '+remaining.slice(0,3).map(c=>c.title).join(' / '):'START 대기 · 실행기·지상 상태 확인';
if(s.telemetry_fresh&&['TAKING_OFF','HANDOFF','MOVING','DWELL','RETURNING','HOLD_READY'].includes(t.flight_state)&&t.observation_status){const o=t.observation_status;if(o.progress_held)by('readiness').textContent='관측 확인 대기 · 현재 목표 유지 · 미션 완료 판정 보류';else if(o.warnings?.length)by('readiness').textContent='상태 보고 지연 기록 중 · 미션 계속';}
if(s.storage?.writer?.error)by('readiness').textContent='기록 저장 오류 · 새 START 차단 · 착륙 요청 가능';
if(t.command_record_pending)by('readiness').textContent='명령 기록 저장 중 · 현재 위치 제어 유지';
if(t.record_pending)by('readiness').textContent='착륙 확인 완료 · 완료 기록 저장 중';
by('checks').replaceChildren();by('observations').replaceChildren();for(const c of checks){const row=document.createElement('div');row.textContent=(c.passed?'✓ ':c.required===false?'미확인 · ':'대기 · ')+c.title+(c.code==='local_start'&&s.automatic_start_xy_m?' · X '+s.automatic_start_xy_m[0].toFixed(3)+'m / Y '+s.automatic_start_xy_m[1].toFixed(3)+'m':'');by(c.required===false?'observations':'checks').appendChild(row);}
by('state').textContent=JSON.stringify({기체:s.assignment.drone_id,배치:s.assignment.anchor_layout_id,텔레메트리수신:s.telemetry_fresh,상태:t.flight_state,사유:t.flight_reason,시동:t.fc_armed,PX4모드:t.fc_mode,UWB_XYZ:[t.x,t.y,t.current_z_m],Z출처:t.current_z_source,Z지상기준추정:t.current_z_estimated,Z실측유효:t.current_z_trusted,XYZ실측유효:t.xyz_valid,PX4_ENU:t.px4_position_enu_m,자동출발점_창고XY:s.automatic_start_xy_m,고정복귀점_창고XY:t.home_xy_m,착륙확인:t.landing_verified,출발점복귀확인:t.home_verified,비행결과:t.flight_outcome,작업결과:t.work_outcome,스캔결과:t.scan_results,요청검사:t.target_validation,웹저장:s.storage,실행기저장:t.storage},null,2);
if(++polls%6===0)await refreshCatalog();
}catch(e){by('takeoff').disabled=true;by('uwbpolicy').textContent='연결 끊김 · 적용 상태 확인 대기';by('start').disabled=true;by('arm').disabled=true;by('land').disabled=true;by('home').disabled=true;by('runstate').textContent='연결 대기 · 이전 미션을 자동 실행하지 않습니다.';by('readiness').textContent='웹 연결 끊김 · START 대기';}setTimeout(update,500);}
refreshCatalog().catch(e=>{notice.textContent=e.message;});update();
</script></html>'''.encode('utf-8')
