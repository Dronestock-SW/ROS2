# 원격 Codex CLI 사용자 경로 복구
2026-10-12 사용자 요청으로 복구한 접속 기록이다.
원격 CLI 미설치 메시지의 원인과 범위를 확인할 때 읽는다.

`pgyxn` 로그인 셸에서 Codex 실행을 복구했다.
CLI `0.160.0`과 기존 ChatGPT 로그인 상태를 확인했다.
Codex 앱의 재연결 화면은 확인하지 않았다.

| 항목 | 관측·변경 |
|---|---|
| 원격 대상 | `100.110.163.94`, 사용자 `pgyxn` |
| 원인 | `~/.local/bin/codex`가 사라진 VS Code 확장 경로를 가리킴 |
| 이전 확장 버전 | `26.928.40906` |
| 현재 발견한 확장 버전 | `26.930.51102` |
| 보존한 이전 링크 | `~/.local/bin/codex.before-20261012-vertical-followup` |
| 복사한 실행 파일 | `~/.local/share/codex-cli/0.160.0/codex` |
| 복구한 링크 | `~/.local/bin/codex` → 위 실행 파일 |
| 실행 파일 SHA-256 | `50b06603bdcdac39b714f5c3e68583c002b8ad8779ebfdaaf4932ff016b379c0` |
| 확인 | 로그인 셸에서 `codex --version`, `codex login status` |

확장 디렉터리와 복사본의 해시는 일치했다.
사용자 경로만 변경했다.
`/usr/local/bin/codex`는 다른 계정 경로를 가리켰다.
그 전역 링크는 변경하지 않았다.
비로그인 SSH 셸의 PATH에는 `~/.local/bin`이 없었다.
로그인 셸 확인을 앱 재연결 성공으로 확대하지 않는다.
로그인 자격 정보는 문서나 저장소에 기록하지 않았다.
실기 서비스·ROS 설정·비행 설정은 바꾸지 않았다.

원본 회수에는 승인된 소유자 계정을 사용했다.
대상 capture와 기존 분석기는 찾지 못했다.
결과는 [LiDAR 후속 보고서](lidar_vertical_followup_20261012.md)에 적었다.

CLI 사용 형태는 [OpenAI 공식 문서](https://developers.openai.com/codex/cli)를 참고했다.
이번 장애 원인은 실제 원격 파일과 링크로 확인했다.
