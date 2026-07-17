# STS3215 custom gripper test

새로 제작한 그리퍼를 기존 GELLO 코드와 분리해서 점검하기 위한 폴더입니다.
첫 단계는 STS3215의 ID 확인 및 변경입니다.

## 준비

- URT-2(또는 호환 어댑터)에 **STS3215 한 개만** 연결합니다.
- 모터 전원과 어댑터의 GND가 올바르게 연결됐는지 확인합니다.
- ID를 바꾸는 동안 다른 서보는 버스에서 분리합니다. 같은 ID를 가진 서보가
  두 개 이상 연결되어 있으면 응답 충돌이나 잘못된 설정이 발생할 수 있습니다.

PowerShell에서 다음을 실행합니다.

```powershell
cd gello_software\custom_gripper_test
python -m pip install -r requirements.txt
python list_ports.py
```

## 1. 현재 ID 찾기

공장 기본 ID가 1일 가능성이 높으므로 먼저 좁은 범위를 스캔합니다.

```powershell
python scan_ids.py --port COM3 --start 0 --end 10
```

찾지 못하면 전체 사용 가능 범위를 스캔할 수 있습니다.

```powershell
python scan_ids.py --port COM3 --start 0 --end 253
```

기본 baudrate는 이 프로젝트에서 사용 중인 `1000000`입니다. 다른 속도라면
`--baudrate 115200`처럼 지정합니다.

## 2. ID 부여

예를 들어 현재 ID 1을 그리퍼용 ID 7로 바꾸려면:

```powershell
python assign_id.py --port COM3 --old-id 1 --new-id 7
```

스크립트가 기존 ID 응답과 새 ID 미사용 상태를 먼저 검사합니다. 화면에 표시된
내용이 맞으면 `CHANGE 1 TO 7`을 정확히 입력해야 EEPROM 쓰기를 진행합니다.
완료 후 전원을 껐다 켜고 다시 스캔해 ID 7이 유지되는지 확인하세요.

```powershell
python scan_ids.py --port COM3 --start 1 --end 10
```

문제가 생기면 우선 포트 이름, 전원, TX/RX 방향, baudrate를 확인하세요.

