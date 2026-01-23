
# Shared Memory Confirmation

1. **`dest` status** = "destruction" - segments marked for deletion but still exist
    
2. **`nattch` (number attached) = 2 or 1** - Some process still has them attached
    
3. **Key `0x00000000`** = Private/unnamed shared memory (Iceoryx uses this)
```bash
watch -n 1 'ipcs -m | grep iceoryx'

Every 1.0s: ipcs -m                                                                                                                                             ted-Predator-PHN16-73: Wed Jan 14 18:45:13 2026


------ Shared Memory Segments --------
key        shmid      owner      perms      bytes      nattch     status
0x00000000 32770      ted        600        15974400   2          dest
0x00000000 32772      ted        600        15974400   2          dest
0x00000000 32773      ted        606        23500620   2          dest
0x00000000 32774      ted        606        23500620   2          dest
0x00000000 8          ted        600        16384      1          dest
0x00000000 163850     ted        600        524288     2          dest
0x00000000 11         ted        600        16384000   2          dest
0x00000000 14         ted        600        524288     2          dest
0x00000000 15         ted        600        16384000   2          dest
0x00000000 32785      ted        606        23500620   2          dest
0x00000000 18         ted        600        524288     2          dest
0x00000000 19         ted        600        8192000    2          dest
0x00000000 20         ted        606        11473920   2          dest
0x00000000 21         ted        606        11473920   2          dest
0x00000000 22         ted        600        16384      1          dest
0x00000000 65559      ted        600        524288     2          dest
0x00000000 32792      ted        606        23500620   2          dest
0x00000000 32793      ted        600        15974400   2          dest
0x00000000 65567      ted        606        23500620   2          dest
0x00000000 65568      ted        606        23500620   2          dest
0x00000000 65569      ted        600        15974400   2          dest
0x00000000 40         ted        606        11750310   2          dest
0x00000000 41         ted        606        11750310   2          dest
0x00000000 42         ted        600        8192000    2          dest
0x00000000 32812      ted        600        15974400   2          dest
0x00000000 32813      ted        606        23500620   2          dest
0x00000000 32814      ted        606        23500620   2          dest
```


```bash
# Iceoryx introspection (if installed)
iox-introspection-client
# or
iox-list

# This will show all shared memory ports and their status
```


### Clean Up Shared Memory
```bash
# First, make sure ALL Iceoryx processes are dead
pkill -9 iox
pkill -9 RouDi
pkill -f iceoryx

# Now clean up
ipcrm --all=shm
# OR
for shm in $(ipcs -m | awk '$3 == "'$USER'" {print $2}'); do ipcrm -m $shm; done
```


# Iceronyx

## Debug Mode

```
./iox-roudi -l debug
```