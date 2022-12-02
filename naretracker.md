나래트래커 (NareTracker-V1)
==========================

# 준비 사항
- PC가 Wi-Fi 라우터(공유기)(Wi-Fi 4, 802.11N)와 같은 네트워크상에 존재해야 함. 혹은, PC가 직접 Wi-Fi를 사용할 수 있어야 함.
  - 퀘스트 사용자의 경우 에어링크를 사용하고 계시다면 Wi-Fi와 같은 환경에 있다고 생각하시면 됩니다.
- PC에 SteamVR이 설치되어 있고, SteamVR상에서 헤드셋으로 인식하는 VR기기가 있어야 함.
- 안드로이드 혹은 iOS가 탑재된 Wi-Fi를 사용가능한 기기(스마트폰)가 있어야 함. 혹은 PC가 직접 Wi-Fi를 사용할 수 있어야 함.
  - PC가 라우터와 연결되어있지 않은 경우 핫스팟 기능을 이용하면 됩니다.
  
  ## 자기장 환경 확인하기
- 휴대폰에서 적당한 나침반 어플을 다운로드, 혹은 물리적인 나침반을 사용해서 방 안 플레이구역 곳곳이 항상 북쪽을 가리키는지 확인합니다.


# 기초 세팅
## SlimeVR Server 설치
- [SlimeVR Server](https://github.com/SlimeVR/SlimeVR-Server/releases)를 다운받아 설치합니다. 초보 사용자는 [Windows web installer](https://github.com/SlimeVR/SlimeVR-Installer/releases/latest/download/slimevr_web_installer.exe)를 이용하는 것이 좋습니다.
## 기기에 Wi-Fi 정보 입력 (PC)
- PC가 직접 Wi-Fi를 사용할 수 있어야 합니다. 그렇지 않은 경우 아래 모바일로 다시 시도해주세요.
- SlimeVR Server를 실행합니다.
- Lets get setup을 누릅니다.
- 모든 기기의 전원을 켭니다.
- SSID는 Wi-Fi의 이름, Password는 Wi-Fi의 비밀번호입니다.
- Submit! 버튼을 누릅니다.
- 기기가 잘 연결되었는지 확인하고, 할당되지 않았다면 기기의 전원을 내렸다 다시 올려봅니다.
## 기기에 Wi-Fi 정보 입력 (모바일)
- 안드로이드 혹은 iOS에서 ESP smartconfig을 검색하여 다운받습니다.
- ESP smartconfig을 실행합니다. 2.4Ghz Wi-Fi 4 (5Ghz가 아닌) 네트워크에 연결합니다.
- 기기의 전원을 켭니다.
- password란에 Wi-Fi 네트워크의 비밀번호를 입력합니다.
- Confirm을 누릅니다.
- SlimeVR Server의 Lets get setup에서 wifi settings란이 뜨면 Skip wifi settings를 누르거나 SSID란에 아무거나 입력하고 Submit!버튼을 누릅니다.
- 기기가 잘 연결되었는지 확인하고, 할당되지 않았다면 기기의 전원을 내렸다 다시 올려봅니다.
## SlimeVR Server에 기기 정보 맵핑
- 트래커를 할당할 부위를 클릭하고, 기기를 흔들어봅니다. 반짝이는 이름을 클릭합니다.
# 몸에 장착하기
- 메인 모듈은 위를 바라보게, 보조 모듈은 아래를 바라보게 설정합니다.
# 실행 시

# 문제 해결
## 2.4Ghz에 연결했는데, 연결되지 않을 경우
- Wi-Fi 6 (802.11ax) 라우터의 경우 2.4Ghz대역에서도 Wi-Fi 6 모드로 작동하기 때문에, 라우터의 설정을 통해 802.11n모드를 활성화해야 합니다. 혹은 기기의 전원을 내렸다 다시 올려보시기 바랍니다.