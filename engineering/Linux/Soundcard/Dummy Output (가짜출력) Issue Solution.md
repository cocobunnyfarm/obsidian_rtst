
# Main Points or Commands Reference



# 해결과정 대략 정리


2026년 1월, 최신 인텔 Arrow Lake 기반 노트북(Predator PHN16-73)에서 발생한 **Ubuntu 22.04 사운드 가짜출력(Dummy Output) 문제**에 대한 기술적 정리 자료입니다.

---

1. 문제 정의

- **현상:** Ubuntu 22.04 설치 후 사운드 출력 장치가 '가짜출력(Dummy Output)'으로만 표시되며 소리가 나지 않음.
- **원인:** OS 버전(22.04)이 하드웨어 출시 시점보다 오래되어, 최신 사운드 칩셋(Intel Arrow Lake DSP)을 구동하기 위한 **정규 펌웨어 바이너리 파일**이 시스템에 존재하지 않음.

---

2. 핵심 배경 지식 및 계층 구조

리눅스 사운드 시스템은 아래와 같은 **프랙탈(Fractal) 구조**의 추상화 계층을 가집니다.

|계층|구성 요소|역할 및 인터페이스|
|---|---|---|
|**Application**|브라우저, 음악 앱|유저 API (libpulse 등) 호출|
|**Sound Server**|**PulseAudio / PipeWire**|여러 앱의 소리를 **소프트웨어 믹싱**. 하드웨어가 없으면 가짜(Dummy) Sink 생성|
|**Kernel (ALSA)**|**snd-sof-pci-intel-mtl**|하드웨어를 추상화한 **로우레벨 드라이버**. 펌웨어를 하드웨어에 주입하는 주체|
|**Hardware**|**Intel DSP (Audio Engine)**|실제 연산 장치. 부팅 시 **휘발성 펌웨어**를 주입받아야 뇌가 깨어남|

- **펌웨어(Firmware)의 관리:** 현대 리눅스는 하드웨어 ROM에 코드를 고정하지 않고, `/lib/firmware` 경로에 바이너리 파일로 보관합니다. 커널 드라이버가 로드될 때 이 파일을 읽어 하드웨어의 SRAM으로 **DMA(Direct Memory Access)** 방식을 통해 쏴줍니다.
- **토폴로지(Topology, .tplg):** 펌웨어가 '뇌'라면 토폴로지는 '신경망 지도'입니다. 각 제조사(Acer, 삼성 등)마다 다른 스피커/마이크 배선을 드라이버에 알려주는 설계도 파일입니다.
- **Zero-copy:** 앱에서 하드웨어까지 데이터가 전달될 때, CPU 복사를 최소화하고 **공유 메모리(mmap)**와 주소 참조(Handle)만으로 데이터를 전달하는 최적화 기법입니다.

---

3. 문제 파악을 위한 주요 커맨드 및 로그 분석

문제를 진단할 때는 **Bottom-Up(하드웨어부터 상위로)** 방식을 사용합니다.

1. **물리 장치 확인:** `lspci -v | grep -i audio`
    - NVIDIA(HDMI) 외에 `Intel Multimedia audio controller`가 있는지 확인.
2. **커널 드라이버 인식 확인:** `aplay -l`
    - 장치 목록에 내장 스피커가 없다면 커널/드라이버 단계의 문제임.
3. **결정적 에러 로그 확인:** `sudo dmesg | grep -iE "snd|sof|audio"`
    - **판단 지점:** `sof-audio-pci-intel-mtl ...: Firmware file: .../sof-arl-s.ri not found`
    - 이 로그를 통해 드라이버는 준비되었으나 **주입할 데이터(펌웨어)가 없음**을 확진.

아래에 보면 가장 중요한 단서들이 있음:
```bash
[    4.280597] sof-audio-pci-intel-mtl 0000:80:1f.3: SOF firmware and/or topology file not found.
[    4.280599] sof-audio-pci-intel-mtl 0000:80:1f.3: Supported default profiles
[    4.280599] sof-audio-pci-intel-mtl 0000:80:1f.3: - ipc type 1 (Requested):
[    4.280600] sof-audio-pci-intel-mtl 0000:80:1f.3:  Firmware file: intel/sof-ipc4/arl-s/sof-arl-s.ri
[    4.280600] sof-audio-pci-intel-mtl 0000:80:1f.3:  Topology file: intel/sof-ace-tplg/sof-hda-generic-4ch.tplg
```

```bash
sudo dmesg | grep -iE "snd|sof|audio"
[sudo] password for ted: 
[    0.040308] software IO TLB: area num 32.
[    0.398383] pps_core: Software ver. 5.3.6 - Copyright 2005-2007 Rodolfo Giometti <giometti@linux.it>
[    0.435530] PCI-DMA: Using software bounce buffering for IO (SWIOTLB)
[    0.435531] software IO TLB: mapped [mem 0x0000000048ced000-0x000000004cced000] (64MB)
[    0.643645] integrity: Loaded X.509 cert 'Microsoft Windows Production PCA 2011: a92902398e16c49778cd90f99e4f9ae17c55af53'
[    0.643665] integrity: Loaded X.509 cert 'Microsoft Corporation: Windows UEFI CA 2023: aefc5fbbbe055d8f8daa585473499417ab5a5272'
[    0.643671] integrity: Loaded X.509 cert 'Microsoft Corporation UEFI CA 2011: 13adbf4309bd82709c8cd54f316ed522988a1bd4'
[    0.643676] integrity: Loaded X.509 cert 'Microsoft UEFI CA 2023: 81aa6b3244c935bce0d6628af39827421e32497d'
[    2.740200] snd_hda_intel 0000:01:00.1: enabling device (0000 -> 0002)
[    2.740361] snd_hda_intel 0000:01:00.1: Disabling MSI
[    2.740365] snd_hda_intel 0000:01:00.1: Handle vga_switcheroo audio client
[    2.742914] snd_hda_intel 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    2.742928] snd_hda_intel 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    3.044496] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    3.044521] sof-audio-pci-intel-mtl 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    3.044684] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if 0x040100
[    3.053870] snd_hda_intel 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    3.053881] snd_hda_intel 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    3.053891] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    3.053895] sof-audio-pci-intel-mtl 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    3.054022] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if 0x040100
[    3.093096] snd_hda_intel 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    3.093106] snd_hda_intel 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    3.093111] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    3.093114] sof-audio-pci-intel-mtl 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    3.093217] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if 0x040100
[    4.241612] snd_hda_intel 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.241621] snd_hda_intel 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.241628] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.241630] sof-audio-pci-intel-mtl 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.241710] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if 0x040100
[    4.242593] snd_hda_intel 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.242600] snd_hda_intel 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.242603] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.242606] sof-audio-pci-intel-mtl 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.242680] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if 0x040100
[    4.243064] snd_hda_intel 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.243068] snd_hda_intel 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.243071] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.243073] sof-audio-pci-intel-mtl 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.243140] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if 0x040100
[    4.245293] sof-audio-pci-intel-mtl 0000:80:1f.3: bound 0000:00:02.0 (ops i915_audio_component_bind_ops [i915])
[    4.252303] sof-audio-pci-intel-mtl 0000:80:1f.3: use msi interrupt mode
[    4.280523] sof-audio-pci-intel-mtl 0000:80:1f.3: hda codecs found, mask 5
[    4.280525] sof-audio-pci-intel-mtl 0000:80:1f.3: using HDA machine driver skl_hda_dsp_generic now
[    4.280528] sof-audio-pci-intel-mtl 0000:80:1f.3: DMICs detected in NHLT tables: 4
[    4.280597] sof-audio-pci-intel-mtl 0000:80:1f.3: SOF firmware and/or topology file not found.
[    4.280599] sof-audio-pci-intel-mtl 0000:80:1f.3: Supported default profiles
[    4.280599] sof-audio-pci-intel-mtl 0000:80:1f.3: - ipc type 1 (Requested):
[    4.280600] sof-audio-pci-intel-mtl 0000:80:1f.3:  Firmware file: intel/sof-ipc4/arl-s/sof-arl-s.ri
[    4.280600] sof-audio-pci-intel-mtl 0000:80:1f.3:  Topology file: intel/sof-ace-tplg/sof-hda-generic-4ch.tplg
[    4.280601] sof-audio-pci-intel-mtl 0000:80:1f.3: Check if you have 'sof-firmware' package installed.
[    4.280601] sof-audio-pci-intel-mtl 0000:80:1f.3: Optionally it can be manually downloaded from:
[    4.280602] sof-audio-pci-intel-mtl 0000:80:1f.3:    https://github.com/thesofproject/sof-bin/
[    4.281864] sof-audio-pci-intel-mtl 0000:80:1f.3: error: sof_probe_work failed err: -2
```
---

4. 문제 원인 및 해결 전략

- **문제 핵심:** 커널 모듈(`mtl`용 범용 드라이버)은 최신 칩셋 ID를 알고 있어 로드되었으나, 정작 칩셋에 넣어줄 전용 바이너리(`arl-s.ri`)가 `/lib/firmware`에 없어서 DSP 부팅에 실패한 것임.
- **해결 전략:** 커널 모듈이 찾는 경로(`/lib/firmware/intel/sof-ipc4/arl-s/`)를 생성하고, 해당 하드웨어 규격에 맞는 **진짜 펌웨어와 토폴로지 파일**을 상위 OS 버전에서 가져와 강제로 이식함.


> [!Important] 상위 커널 및 우분투 버전에서 펌웨어를 가져와도 되는 이유
> 1) 구형 우분투 및 커널 버전에 최신 인텔 펌웨어가 없음
> 2) 펌웨어는 하드웨어에 종속되지, 커널 등의 운영체제와 종속되는 개념이 아님. 따라서 최신 버전의 우분투 apt 저장소 등에서 하드웨어에 맞는 펌웨어를 제공한다면 그대로 가져와서 적용하면 됨.


---

5. 실전 해결 단계 (24.04/25.10 패키지 활용)

6. **패키지 검색:** Ubuntu Packages 사이트에서 `firmware-sof-signed` 키워드로 검색. 현재 버전(22.04)이 아닌 최신 버전(예: **Questing 25.10**)의 패키지 정보를 확인.

`firmware-sof-signed` 을 questing 25.10 패키지에서 검색

[검색결과 링크](https://packages.ubuntu.com/questing/firmware-sof-signed)

[아래와 정보가 나와 있는 패키지 정보 링크](https://packages.ubuntu.com/questing/all/firmware-sof-signed/download)

```
## Download Page for firmware-sof-signed_2025.05.1-1_all.deb

If you are running Ubuntu, it is strongly suggested to use a package manager like [aptitude](https://packages.ubuntu.com/questing/aptitude) or [synaptic](https://packages.ubuntu.com/questing/synaptic) to download and install packages, instead of doing so manually via this website.

You should be able to use any of the listed mirrors by adding a line to your /etc/apt/sources.list like this:

deb http://_cz.archive.ubuntu.com/ubuntu_ questing main 

Replacing _cz.archive.ubuntu.com/ubuntu_ with the mirror in question.

You can download the requested file from the pool/main/f/firmware-sof/ subdirectory at any of these sites:

_North America_

- [mirrors.kernel.org/ubuntu](http://mirrors.kernel.org/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [ftp.osuosl.org/pub/ubuntu](http://ftp.osuosl.org/pub/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [lug.mtu.edu/ubuntu](http://lug.mtu.edu/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [ubuntu.mirrors.tds.net/ubuntu](http://ubuntu.mirrors.tds.net/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [ubuntu.secs.oakland.edu](http://ubuntu.secs.oakland.edu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mirror.mcs.anl.gov/pub/ubuntu](http://mirror.mcs.anl.gov/pub/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mirrors.cat.pdx.edu/ubuntu](http://mirrors.cat.pdx.edu/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [ubuntu.cs.utah.edu/ubuntu](http://ubuntu.cs.utah.edu/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [ftp.ussg.iu.edu/linux/ubuntu](http://ftp.ussg.iu.edu/linux/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mirrors.xmission.com/ubuntu](http://mirrors.xmission.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mirrors.cs.wmich.edu/ubuntu](http://mirrors.cs.wmich.edu/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [gulus.USherbrooke.ca/pub/distro/ubuntu](http://gulus.usherbrooke.ca/pub/distro/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)

_Asia_

- [kr.archive.ubuntu.com/ubuntu](http://kr.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [th.archive.ubuntu.com/ubuntu](http://th.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mirror.lupaworld.com/ubuntu](http://mirror.lupaworld.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [kambing.vlsm.org/ubuntu](http://kambing.vlsm.org/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [ubuntu.mithril-linux.org/archives](http://ubuntu.mithril-linux.org/archives/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mirror.in.th/ubuntu](http://mirror.in.th/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mirror.rootguide.org/ubuntu](http://mirror.rootguide.org/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)

_Africa_

- [za.archive.ubuntu.com/ubuntu](http://za.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)

_Europe_

- [cz.archive.ubuntu.com/ubuntu](http://cz.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [de.archive.ubuntu.com/ubuntu](http://de.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [dk.archive.ubuntu.com/ubuntu](http://dk.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [es.archive.ubuntu.com/ubuntu](http://es.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [fr.archive.ubuntu.com/ubuntu](http://fr.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [ge.archive.ubuntu.com/ubuntu](http://ge.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [gr.archive.ubuntu.com/ubuntu](http://gr.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [hr.archive.ubuntu.com/ubuntu](http://hr.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mt.archive.ubuntu.com/ubuntu](http://mt.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [nl.archive.ubuntu.com/ubuntu](http://nl.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [no.archive.ubuntu.com/ubuntu](http://no.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [se.archive.ubuntu.com/ubuntu](http://se.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [yu.archive.ubuntu.com/ubuntu](http://yu.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)

- [nz.archive.ubuntu.com/ubuntu](http://nz.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [nz2.archive.ubuntu.com/ubuntu](http://nz2.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [ftp.iinet.net.au/pub/ubuntu](http://ftp.iinet.net.au/pub/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mirror.optus.net/ubuntu](http://mirror.optus.net/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [ftp.filearena.net/pub/ubuntu](http://ftp.filearena.net/pub/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)
- [mirror.pacific.net.au/linux/ubuntu](http://mirror.pacific.net.au/linux/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb)

If none of the above sites are fast enough for you, please see our [complete mirror list](https://launchpad.net/ubuntu/+archivemirrors).

Note that in some browsers you will need to tell your browser you want the file saved to a file. For example, in Firefox or Mozilla, you should hold the Shift key when you click on the URL.

### More information on firmware-sof-signed_2025.05.1-1_all.deb:

|   |   |
|---|---|
|Exact Size|1695690 Byte (1.6 MByte)|
|MD5 checksum|21acc5d981a834a070c06c95043e267d|
|SHA1 checksum|e5e1d4b4387070c4a569f4fc2187d61e5073d54c|
|SHA256 checksum|f346f12f4d3e20150a5d7c1d4f5a9910052ca0636fb557c13a6704b5fb22dea7|
```




> [!Important] 다운로드 경로 찾음 (이건 Ubuntu packages에서 안알려줘서 유추해야 함)
> `wget http://kr.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb`


7. **수동 다운로드:** `apt` 저장소를 추가하지 않고, `wget`을 사용하여 `.deb` 패키지 파일만 직접 다운로드.

```bash
wget http://kr.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb
--2026-01-17 01:46:30--  http://kr.archive.ubuntu.com/ubuntu/pool/main/f/firmware-sof/firmware-sof-signed_2025.05.1-1_all.deb
Resolving kr.archive.ubuntu.com (kr.archive.ubuntu.com)... 210.117.237.2, 2001:320:237::2
Connecting to kr.archive.ubuntu.com (kr.archive.ubuntu.com)|210.117.237.2|:80... connected.
HTTP request sent, awaiting response... 200 OK
Length: 1695690 (1.6M) [application/octet-stream]
Saving to: ‘firmware-sof-signed_2025.05.1-1_all.deb’

firmware-sof-signed_2025.05.1 100%[=================================================>]   1.62M  9.97MB/s    in 0.2s    

2026-01-17 01:46:31 (9.97 MB/s) - ‘firmware-sof-signed_2025.05.1-1_all.deb’ saved [1695690/1695690]

```

    
    
5.  deb 폴더 추출하기 및 폴더 복사하기

```bash
ls
firmware-sof-signed_2025.05.1-1_all.deb
mkdir sof-extract

dpkg-deb -x firmware-sof-signed_2025.05.1-1_all.deb ./sof-extract

ls
firmware-sof-signed_2025.05.1-1_all.deb  sof-extract
cd sof-extract/

```


아래 파일들이 우리가 원하는 것. 즉, 여기에서 `arl-s` 폴더만 리눅스 루트 펌웨어 경로로 복사해주면 된다.
```bash
/usr/lib/firmware/intel/sof-ipc4/arl-s/community/sof-arl-s.ri
/usr/lib/firmware/intel/sof-ipc4/arl-s/intel-signed/sof-arl-s.ri
/usr/lib/firmware/intel/sof-ipc4/arl-s/sof-arl-s.ri
```

> [!Warning] 토폴로지 파일은 외부에서 옮기지 않는다. 
> 토폴로지 파일은 acer가 제공한 파일이 따로 펌웨어 폴더에 존재함. acer가 토포롤지를 정했기 때문에 당연히 이 파일을 써야 함.

`arl-s`  폴더를 찾아서 `sof-ipc4` 폴더에 복사해주면 된다.
```bash
sudo cp -r ./arl-s /lib/firmware/intel/sof-ipc4/
[sudo] password for ted: 
```


5. **적용:** 재부팅 또는 커널 모듈 리로드.
    
     재부팅해주면 커널 드라이버가 새로 받은 firmware를 사운드카드에 로드시켜준다. 그러면 문제 해결 끝.  제대로 됐는지는 아래의 커널 로그를 확인해본다:
	 
    



> [!Important] 문제 해결!
> ` sof-audio-pci-intel-mtl` 이 펌웨어 및 펌웨어 토폴로지를 찾아서 적용한 걸 볼 수 있다. 

```bash
sudo dmesg | grep -iE "snd|sof|audio"
[sudo] password for ted: 
[    0.040361] software IO TLB: area num 32.
[    0.401406] pps_core: Software ver. 5.3.6 - Copyright 2005-2007 Rodolfo Giometti <giometti@linux.it>
[    0.441529] PCI-DMA: Using software bounce buffering for IO (SWIOTLB)
[    0.441530] software IO TLB: mapped [mem 0x0000000048ced000-0x000000004cced000] (64MB)
[    0.648265] integrity: Loaded X.509 cert 'Microsoft Windows Production PCA 2011: a92902398e16c49778cd90f99e4f9ae17c55af53'
[    0.648282] integrity: Loaded X.509 cert 'Microsoft Corporation: Windows UEFI CA 2023: aefc5fbbbe055d8f8daa585473499417ab5a5272'
[    0.648302] integrity: Loaded X.509 cert 'Microsoft Corporation UEFI CA 2011: 13adbf4309bd82709c8cd54f316ed522988a1bd4'
[    0.648317] integrity: Loaded X.509 cert 'Microsoft UEFI CA 2023: 81aa6b3244c935bce0d6628af39827421e32497d'
[    3.046971] snd_hda_intel 0000:01:00.1: enabling device (0000 -> 0002)
[    3.047168] snd_hda_intel 0000:01:00.1: Disabling MSI
[    3.047173] snd_hda_intel 0000:01:00.1: Handle vga_switcheroo audio client
[    3.047273] snd_hda_intel 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    3.047288] snd_hda_intel 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    3.115300] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    3.115320] sof-audio-pci-intel-mtl 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    3.115490] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if 0x040100
[    4.319429] snd_hda_intel 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.319446] snd_hda_intel 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.319452] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.319456] sof-audio-pci-intel-mtl 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.319652] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if 0x040100
[    4.320370] snd_hda_intel 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.320375] snd_hda_intel 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.320378] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if info 0x040100
[    4.320384] sof-audio-pci-intel-mtl 0000:80:1f.3: Digital mics found on Skylake+ platform, using SOF driver
[    4.320458] sof-audio-pci-intel-mtl 0000:80:1f.3: DSP detected with PCI class/subclass/prog-if 0x040100
[    4.325611] sof-audio-pci-intel-mtl 0000:80:1f.3: bound 0000:00:02.0 (ops i915_audio_component_bind_ops [i915])
[    4.332526] sof-audio-pci-intel-mtl 0000:80:1f.3: use msi interrupt mode
[    4.349883] sof-audio-pci-intel-mtl 0000:80:1f.3: hda codecs found, mask 5
[    4.349886] sof-audio-pci-intel-mtl 0000:80:1f.3: using HDA machine driver skl_hda_dsp_generic now
[    4.349890] sof-audio-pci-intel-mtl 0000:80:1f.3: DMICs detected in NHLT tables: 4
[    4.352156] sof-audio-pci-intel-mtl 0000:80:1f.3: Firmware paths/files for ipc type 1:
[    4.352159] sof-audio-pci-intel-mtl 0000:80:1f.3:  Firmware file:     intel/sof-ipc4/arl-s/sof-arl-s.ri
[    4.352160] sof-audio-pci-intel-mtl 0000:80:1f.3:  Firmware lib path: intel/sof-ipc4-lib/arl-s
[    4.352160] sof-audio-pci-intel-mtl 0000:80:1f.3:  Topology file:     intel/sof-ace-tplg/sof-hda-generic-4ch.tplg
[    4.352617] sof-audio-pci-intel-mtl 0000:80:1f.3: Loaded firmware library: ADSPFW, version: 2.13.0.1
[    4.662540] sof-audio-pci-intel-mtl 0000:80:1f.3: Booted firmware version: 2.13.0.1
[    4.687105] sof-audio-pci-intel-mtl 0000:80:1f.3: Topology: ABI 3:29:0 Kernel ABI 3:23:0
[    4.717626] snd_hda_codec_realtek ehdaudio0D0: autoconfig for ALC245: line_outs=1 (0x14/0x0/0x0/0x0/0x0) type:speaker
[    4.717628] snd_hda_codec_realtek ehdaudio0D0:    speaker_outs=0 (0x0/0x0/0x0/0x0/0x0)
[    4.717629] snd_hda_codec_realtek ehdaudio0D0:    hp_outs=1 (0x21/0x0/0x0/0x0/0x0)
[    4.717630] snd_hda_codec_realtek ehdaudio0D0:    mono: mono_out=0x0
[    4.717630] snd_hda_codec_realtek ehdaudio0D0:    inputs:
[    4.774381] input: sof-hda-dsp Headphone as /devices/pci0000:80/0000:80:1f.3/skl_hda_dsp_generic/sound/card1/input21
[    4.774430] input: sof-hda-dsp HDMI/DP,pcm=3 as /devices/pci0000:80/0000:80:1f.3/skl_hda_dsp_generic/sound/card1/input22
[    4.774452] input: sof-hda-dsp HDMI/DP,pcm=4 as /devices/pci0000:80/0000:80:1f.3/skl_hda_dsp_generic/sound/card1/input23
[    4.774472] input: sof-hda-dsp HDMI/DP,pcm=5 as /devices/pci0000:80/0000:80:1f.3/skl_hda_dsp_generic/sound/card1/input24
```

---

6. 추가 팁 및 주의사항

- **심볼릭 링크(`ln -s`)의 한계:** 구형 세대(MTL)의 펌웨어를 이름만 바꿔서(ARL) 속이는 방식은 하드웨어의 버전 체크/타임아웃 에러로 인해 실패할 확률이 높음. 반드시 **전용 바이너리**를 사용해야 함.

이 가이드는 최신 하드웨어를 구형 리눅스 환경에서 가동해야 하는 모든 상황에 적용 가능한 **표준적인 드라이버 트러블슈팅 모델**입니다. Intel SOF Project 공식 문서에서 더 자세한 사양을 참고할 수 있습니다.