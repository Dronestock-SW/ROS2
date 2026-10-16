# 40 Hz A/B TDMA 실험 태그

Tag A는 `tag_a_tdma40` (주소 5), B는 `tag_b_tdma40` (주소 6) 환경으로 빌드한다.
A1의 SYNC를 받은 frame에서만 한 번 ranging한다. 기존 single-tag 45 Hz 이미지와 구분한다.
공통 설정은 `../common/UwbTdma40.h`, 절차와 검증 결과는 실험 패키지 루트 README를 따른다.
`PACKET_CONTRACT.md`의 RAW 계약은 유지하며 별도 `uwb_tdma_epoch` 진단 JSON을 출력한다.

LCD는 새 4/4 측정 완료 후 TAG A/B, 1.5초 동안 완료가 없으면 LOST CONNECT다.
OLED는 별도 코어에서 상태가 달라질 때만 갱신한다.
앵커 임의 배치로 통신률을 볼 때는 패키지 루트 RADIO_TEST.md를 따른다.
