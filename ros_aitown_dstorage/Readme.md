```Bash
rostopic info /set_id
# ros_aitown_dstorage/SetID
```

```Bash
rosmsg show ros_aitown_dstorage/SetID
# uint64 id
# uint8[] data
```

```Bash
rosservice list
# /get_id
# /new_id
```

```Bash
rosservice type /get_id
# ros_aitown_dstorage/GetID
```

```Bash
# rossrv show ros_aitown_dstorage/GetID
# uint64 id
# ---
# uint8[] data
```

```Bash
rosservice type /new_id
# ros_aitown_dstorage/NewID
```

```Bash
rossrv show ros_aitown_dstorage/NewID
# uint8[] initial_data
# ---
# uint64 id
```
