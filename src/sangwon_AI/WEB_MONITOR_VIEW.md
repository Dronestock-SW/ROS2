# 웹 업데이트 모니터
이 문서는 사용자가 볼 수 있는 읽기 전용 점검 화면을 설명한다.
화면 실행·갱신 주기와 점검 결과의 의미를 확인할 때 읽는다.

주소: http://127.0.0.1:8877/ (사용자 Windows PC에서 실행).
2026-10-04 화면에서 Jetson 보고서 수신, 임무 API 200/계약 1.1,
자율 상태 API 404, 한국 시간 점검 이력 표시를 확인했다.

| 구성 | 동작 |
|---|---|
| Codex heartbeat `drone5-api` | 30분 간격 API·Git 점검, 의미 있는 변화 알림 |
| Jetson `ops/check_web_contract.py` | GET만 실행, 상태·필드 목록·최근 72개 점검 이력 저장 |
| Windows `ops/web_monitor_view.py` | localhost만 listen, SSH로 저장된 보고서 읽기 |
| 화면 `ops/web_monitor.html` | 10초마다 결과 재조회, API·마지막 시각·이력 표시 |

화면 새로고침은 실제 웹 재점검이나 비행 명령을 보내지 않는다.
응답 버전이 맞아도 전체 호환/비행 준비 완료로 표시하지 않는다.
Jetson 읽기 실패 또는 점검 시각이 40분 넘게 오래되면 최신 상태 확인 필요로 표시한다.
API 상태와 Git 커밋 메타데이터만 다루며 원본 임무·인증키·비밀번호는 읽거나 표시하지 않는다.

화면 서버는 이번 Windows 세션의 배경 프로세스다. Windows 자동 시작 등록은 하지 않았다.
종료/재부팅 후에는 저장소 루트에서 다음 명령으로 다시 실행한다.

```powershell
python src/sangwon_AI/ops/web_monitor_view.py --port 8877
```

같은 포트에서 이미 실행 중이면 추가 실행하지 않고 브라우저 주소만 연다.
화면 서버 종료가 Jetson의 host/core/web 서비스나 Codex 점검 예약을 변경하지 않는다.
예약 실행 여부와 네트워크 연결에 따라 실제 결과 갱신은 지연될 수 있으므로 마지막 점검 시각을 확인한다.

## Git 후속 조회

Git Credential Manager 사용자 인증 후 `ops/check_web_git.py`가 origin/main을 fetch한다.
커밋 변경 시 인증·세션·snapshot·ACK·결과 저장 변경을 대조한다.
checkout·원격 소스·관리자 페이지를 수정하지 않는다.
Git 키 만료 등 3회 연속 실패와 복구만 알리고, 변화 없으면 반복 알림을 하지 않는다.
현재 API 상태 200/1.1 및 autonomy-state 302는 장치 등록·인수 완료를 뜻하지 않는다.
