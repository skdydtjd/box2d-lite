# Box2D-Lite
Box2D-Lite is a small 2D physics engine. It was developed for the [2006 GDC Physics Tutorial](docs/GDC2006_Catto_Erin_PhysicsTutorial.pdf). This is the original version of the larger [Box2D](https://box2d.org) library. The Lite version is more suitable for learning about game physics.

# Building
- Install [CMake](https://cmake.org/)
- Ensure CMake is in the user `PATH`
- Visual Studio 2017: run `build.bat`
- Otherwise: run `build.sh` from a bash shell
- Results are in the build sub-folder

# Build Status
[![Build Status](https://travis-ci.org/erincatto/box2d-lite.svg?branch=master)](https://travis-ci.org/erincatto/box2d-lite)

# 2020 

# 코드 분석 및 추가, 개선한 부분

빙판 시스템, 시간 정지, 모터 시스템, 두 강체를 던지는 기능, 충돌 판정 추가

담당한 구현 부분 -> 빙판 및 시간 정지 기능

빙판 기능 -> include 파일의 Arbiter.h에 마찰력 변수를 조정할 flag 변수 추가,  src 파일의 Arbiter.cpp에 flag변수를 관리 할 if문 추가,  sample  파일의 main.cpp에 키 입력 구문 추가

시간 정지 기능 -> sample 파일에 main.cpp에 물리 시간을 조정할 flag변수 및 키입력 구문, if문 추가
