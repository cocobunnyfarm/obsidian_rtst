


# When iox-roudi has weird issues (like permission mistake with root)

3 locations, that can be deleted if there is conflict:
```
ll  /tmp/iceoryx_rt_* && \
ll  /tmp/iceoryx  && \
ll /dev/shm/iox_*
```


Make sure of the below and then run `rm` command:
- RouDi is stopped
- no iceoryx apps are running
⚠ BUT never delete them while RouDi and apps are running.

```bash
rm -f /tmp/iceoryx_rt_*
rm -rf /tmp/iceoryx
rm -f /dev/shm/iox_*
```