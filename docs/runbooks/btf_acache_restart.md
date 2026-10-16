# A-cache 앵커 변경·재시작 절차
운영 설정 변경 시 캐시와 계산기의 배치를 맞추는 절차다.
앵커를 옮기거나 이전 운영 슬롯으로 복귀할 때 읽는다.

## 앵커 배치 변경

**물리 배치·수신기·B_TF 설정을 함께 맞춘 뒤 재시작한다.**

1. 기체가 지상·DISARM인지 최신 FC 상태로 확인한다.
2. 현재 release·슬롯·설정 파일의 해시를 보존한다.
3. 실제 앵커 좌표와 순서를 측정한다.
4. 수신기의 앵커 설정과 B_TF anchors_xyz_m·layout_id를 맞춘다.
5. 각 실행 경로가 읽는 실제 config 경로를 확인한다.
6. 해당 운영 서비스를 재시작한다.
7. 새 프로세스의 instant_xy_cache에서 builds=1·usable=true를 확인한다.
8. 기하 해시를 새 좌표·순서의 해시와 비교한다.
9. backend·서비스 재시작 횟수·ToF/IMU 수신·기록 오류를 확인한다.
10. 실제 UWB RAW 입력에서 calls 증가·XY 판정·발행을 확인한다.

파일 수정만으로 실행 중인 배치가 바뀌지는 않는다.
이유: 캐시와 기존 native 계산기 모두 시작 때 설정을 읽는다.
ClockMap·모델 reset을 설정 재로딩으로 사용하지 않는다.
관측 노드의 fc_output_enabled=false만으로 전체 라우터가 꺼졌다고 보지 않는다.
실제 발행자·라우터 경로와 비행 명령 여부는 별도로 확인한다.

## 2026-10-15 슬롯 복귀

**기존 슬롯과 native 라이브러리를 함께 보존한다.**

현재 적용 슬롯: `/home/arialhanho/btf-acache-release-20261015`.
이전 슬롯: `/home/arialhanho/evening-20261014-1815`.
서비스: arialhanho 사용자의 `dronestock-manual-observe.service`.
새 슬롯도 이전 슬롯의 libbtf_native.so를 사용한다.
이전 슬롯 삭제 시 새 슬롯까지 기동에 실패할 수 있다.

1. 최신 DISARM·지상 상태를 다시 확인한다.
2. 적용 drop-in을 확인하고 사본을 남긴다.
   `~/.config/systemd/user/dronestock-manual-observe.service.d/zzzz-acache-20261015.conf`
3. 이 파일만 `.conf`가 아닌 보관 이름으로 바꾼다.
4. arialhanho 사용자 systemd에서 daemon-reload를 수행한다.
5. 센서 서비스를 restart한다.
6. 실제 실행 경로가 이전 슬롯인지 확인한다.
7. FC 연결·센서 수신·backend·재시작 횟수를 확인한다.

해당 서비스는 MAVROS·UWB·B_TF·스트림 요청 노드를 재시작한다.
따라서 텔레메트리 연결이 잠시 다시 맺어진다.
새 배포는 SSH와 분리된 사용자 systemd 작업으로 수행했다.
실패·ERR/INT/TERM/HUP 시 복귀·상태 검사를 하도록 구성했다.
실제 적용에서는 복귀가 발생하지 않았다.
복귀 중 재신호를 무시하도록 보완했다.
이 문서는 수행 순서이며 자동 실행 파일이 아니다.
당시 activate.sh 원본은 로컬 배포 증거에 보존했다.
[배포 결과와 식별값](../report/btf_acache_deployment_20261015.md)을 따른다.
