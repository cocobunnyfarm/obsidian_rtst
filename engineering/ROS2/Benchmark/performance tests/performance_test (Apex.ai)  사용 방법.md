
# 사용방법
명령어 구조: `ros2 run performance_test perf_test [옵션]`

| **옵션**                   | **설명**                    | **비고**                               |
| ------------------------ | ------------------------- | ------------------------------------ |
| **`-c`**                 | Communicator(Executor) 선택 | `rclcpp-single-threaded-executor` 추천 |
| **`-m`**                 | 메시지 타입 (데이터 크기)           | `Array1k`, `Array1m`, `Array4m` 등    |
| **`-r`**                 | 발행 주기 (Rate, Hz)          | 예: `-r 100` (100Hz로 발행)              |
| **`--max-runtime`**      | **실행 시간(초)**              | 예: `--max-runtime 60` (60초 후 자동 종료)  |
| **`--shared-memory`**    | 공유 메모리 활성화                | CycloneDDS 사용 시 필수                   |
| **`--zero-copy`**        | 제로 카피 기술 활성화              | SHM의 진정한 성능을 위해 필수                   |
| **`-l` (소문자 L)**         | **로그 파일 저장**              | **가장 중요한 옵션 (파일 이름 지정)**             |
| **`--print-to-console`** | 터미널에 통계 출력                | 로그 저장 중에도 실시간 확인용                    |


# UDP (기본 통신) 테스트
```bash
ros2 run performance_test perf_test \ -c rclcpp-single-threaded-executor \ -m Array1m \ -r 100 \ --max-runtime 30 \ --print-to-console
```


# SHM (Shared Memory) 테스트

`--zero-copy` 는 껐다 켰다 해보면서 확인 필요
```bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 run performance_test perf_test \
  -c rclcpp-single-threaded-executor \
  -m Array1m \
  -r 100 \
  --shared-memory \
  --zero-copy \
  --max-runtime 30 \
  --print-to-console
```

