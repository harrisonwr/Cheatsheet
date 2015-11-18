BeagleBone Setup
===
### Installing New OS
Tutorial: https://www.youtube.com/watch?v=oRGrm8RfGCE

If you try to ssh into your beagle bone after flashing the new image and get a warning: "REMOTE HOST IDENTIFICATION HAS CHANGED", you will need to delete your known host (192.168.7.2) in ~/.ssh/known_host
```python
$ vi ~/.ssh/known_hosts
```

Find the one that says 192.168.7.2 and type to remove that line
```python
dd
```

After ssh into the beagle bone, check the version of your OS
```python
$ uname -a
```

### Setup User Account
Tutorial: https://www.youtube.com/watch?v=QfMAewwyzrc

Setup your username, password and some detail info.
```python
$ adduser <your_new_username>
```

Give your new account sudo privileges
```python
$ adduser <your_new_username> sudo
```

Change your username
```python
# exit all the existing ssh
$ exit
$ ssh 192.168.7.2 -l root
$ usermod -l <new_username> <old_username>

```

### Setup Default Shell
Tutorial: https://www.youtube.com/watch?v=QfMAewwyzrc
```python
$ usermod -s /bin/bash <your_new_username>
```

Login to your beagle bone with your new username
```python
$ ssh <your_new_username>@192.168.7.2
```

See what is your default shell
```python
$ which $SHELL
```

Setup alias
```python
$ vi ~/.bash_profile
```
Example: 'ls' will become 'ls -l'
```python
# ~/.bash_profile
alias ls='ls -l'
```

### Wifi Setup
Make sure there is not wifi configuration in this file, comment them if you see them
```python
$ sudo vi /etc/network/interfaces
```
