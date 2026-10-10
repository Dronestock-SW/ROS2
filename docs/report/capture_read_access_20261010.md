# 실측 기록 읽기 권한 개선

읽기 권한 개선의 적용 기록이다.
기록 접근 거부가 재발할 때 읽는다.

기존 파일과 새 기록의 읽기를 확인했다.
수집기는 arialhanho, 읽는 계정은 bluewak이다.
원인은 기본 ACL 부재였다.
기존 파일 ACL만으로는 새 파일에 권한이 전해지지 않았다.

수집 경로는 `/home/arialhanho/.local/share/dronestock/manual-captures`다.
상위 경로에는 bluewak 통과 권한을 부여했다.
기존 파일에는 읽기 권한을 부여했다.
기록 폴더에는 다음 ACL을 적용했다.

```text
u:bluewak:r-x
d:u::rwx,d:u:bluewak:r-x,d:g::---,d:m::r-x,d:o::---
```

사용자가 관리자 권한으로 적용했다.
상태 파일 교체 뒤에도 읽기에 성공했다.
18:21:13의 새 기록에서도 세 파일을 읽었다.
bluewak의 쓰기 권한은 부여하지 않았다.
서비스 재시작·FC 설정 변경은 없었다.
적용 스크립트는 임시 파일이었다.
재사용 가능한 설치 도구를 추가한 작업은 아니다.

[검증 근거](evidence/uwb_field_20261010/capture_read_access_20261010.json)
