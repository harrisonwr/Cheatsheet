Basic TMUX Tutorials
=======================
###Play with Windows
```python
# start tmux server
$ tmux

# tell tmux to do something
# template: ctrl-b <command>

# open a new window
# you can see which window you are using at the bottom of the terminal
ctrl-b + c

# change the name of a window
# you can type whatever name you want after calling the command
ctrl-b + ,

# change to previous window
ctrl-b + p

# change to next window
ctrl-b + n

# list windows and give you a selectable list to let you choose
ctrl-b + w

```
### Play with Panes
```python

# split the terminal vertically
ctrl-b + %

# split the window horizontally
ctrl-b + "

# give name command to tmux
# examples:
#   1. split-window -> this will let you split the window horizontally
ctrl-b + :

# switch pane
ctrl-b + arrow_key

# close pane, press y after calling the command
ctrl-b + x

# kill everything in tmux
ctrl-b + d

# scroll in the window/pane
ctrl-b + :
setw -g mode-mouse on
```

### Play with Sessions
```python
# create new session
$ tmux new -s backupsession

# attach back to the session
$ tmux attach -t backupsession
```
### More Info
##### http://www.dayid.org/os/notes/tm.html
##### https://danielmiessler.com/study/tmux/
##### https://gist.github.com/henrik/1967800
