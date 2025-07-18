
# ~/.bashrc: executed by bash(1) for non-login shells.
# see /usr/share/doc/bash/examples/startup-files (in the package bash-doc)
# for examples

# If not running interactively, don't do anything
case $- in
    *i*) ;;
      *) return;;
esac

# don't put duplicate lines or lines starting with space in the history.
# See bash(1) for more options
HISTCONTROL=ignoreboth

# append to the history file, don't overwrite it
shopt -s histappend

# for setting history length see HISTSIZE and HISTFILESIZE in bash(1)
HISTSIZE=1000
HISTFILESIZE=2000

# check the window size after each command and, if necessary,
# update the values of LINES and COLUMNS.
shopt -s checkwinsize

# If set, the pattern "**" used in a pathname expansion context will
# match all files and zero or more directories and subdirectories.
#shopt -s globstar

# make less more friendly for non-text input files, see lesspipe(1)
[ -x /usr/bin/lesspipe ] && eval "$(SHELL=/bin/sh lesspipe)"

# set variable identifying the chroot you work in (used in the prompt below)
if [ -z "${debian_chroot:-}" ] && [ -r /etc/debian_chroot ]; then
    debian_chroot=$(cat /etc/debian_chroot)
fi

# set a fancy prompt (non-color, unless we know we "want" color)
case "$TERM" in
    xterm-color|*-256color) color_prompt=yes;;
esac

# uncomment for a colored prompt, if the terminal has the capability; turned
# off by default to not distract the user: the focus in a terminal window
# should be on the output of commands, not on the prompt
#force_color_prompt=yes

if [ -n "$force_color_prompt" ]; then
    if [ -x /usr/bin/tput ] && tput setaf 1 >&/dev/null; then
	# We have color support; assume it's compliant with Ecma-48
	# (ISO/IEC-6429). (Lack of such support is extremely rare, and such
	# a case would tend to support setf rather than setaf.)
	color_prompt=yes
    else
	color_prompt=
    fi
fi

if [ "$color_prompt" = yes ]; then
    PS1='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\]\$ '
else
    PS1='${debian_chroot:+($debian_chroot)}\u@\h:\w\$ '
fi
unset color_prompt force_color_prompt

# If this is an xterm set the title to user@host:dir
case "$TERM" in
xterm*|rxvt*)
    PS1="\[\e]0;${debian_chroot:+($debian_chroot)}\u@\h: \w\a\]$PS1"
    ;;
*)
    ;;
esac

# enable color support of ls and also add handy aliases
if [ -x /usr/bin/dircolors ]; then
    test -r ~/.dircolors && eval "$(dircolors -b ~/.dircolors)" || eval "$(dircolors -b)"
    alias ls='ls --color=auto'
    #alias dir='dir --color=auto'
    #alias vdir='vdir --color=auto'

    alias grep='grep --color=auto'
    alias fgrep='fgrep --color=auto'
    alias egrep='egrep --color=auto'
fi

# colored GCC warnings and errors
#export GCC_COLORS='error=01;31:warning=01;35:note=01;36:caret=01;32:locus=01:quote=01'

# some more ls aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

# Add an "alert" alias for long running commands.  Use like so:
#   sleep 10; alert
alias alert='notify-send --urgency=low -i "$([ $? = 0 ] && echo terminal || echo error)" "$(history|tail -n1|sed -e '\''s/^\s*[0-9]\+\s*//;s/[;&|]\s*alert$//'\'')"'

# Alias definitions.
# You may want to put all your additions into a separate file like
# ~/.bash_aliases, instead of adding them here directly.
# See /usr/share/doc/bash-doc/examples in the bash-doc package.

if [ -f ~/.bash_aliases ]; then
    . ~/.bash_aliases
fi

# enable programmable completion features (you don't need to enable
# this, if it's already enabled in /etc/bash.bashrc and /etc/profile
# sources /etc/bash.bashrc).
if ! shopt -oq posix; then
  if [ -f /usr/share/bash-completion/bash_completion ]; then
    . /usr/share/bash-completion/bash_completion
  elif [ -f /etc/bash_completion ]; then
    . /etc/bash_completion
  fi
fi

undock() {
  if [ -z "$1" ]; then
    echo "Usage: undock <namespace>"
    return 1
  fi
  ros2 action send_goal /robot$1/undock irobot_create_msgs/action/Undock "{}"
}
dock() {
  if [ -z "$1" ]; then
    echo "Usage: dock <namespace>"
    return 1
  fi
  ros2 action send_goal /robot$1/dock irobot_create_msgs/action/Dock "{}"
}
localize() {
  if [ -z "$1" ]; then
    echo "Usage: ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot4 map:=$HOME/first_map.yaml"
    return 1
  fi
  ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot$1 map:=$HOME/first_map.yaml
}
nav() {
  if [ -z "$1" ]; then
    echo "Usage: ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot4"
    return 1
  fi
  ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot$1
}
rv() {
  if [ -z "$1" ]; then
    echo "Usage: ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot4"
    return 1
  fi
  ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot$1
}

#-------------------- 최초 자동 실행 스크립트 -------------------- #

# ROS2 설정
echo "ROS2 humble is activated!"
source /opt/ros/humble/setup.bash
# TurtleBot4 workspace 설정
echo "turtlebot4_ws is activated!"
source ~/turtlebot4_ws/install/setup.bash
# TurtleBot4 Discovery 서버 설정
echo "turtlebot4 discovery is activated!"
source /etc/turtlebot4_discovery/setup.bash

echo "source install!"
source ~/rokey_ws/install/setup.bash

#-------------------- 명령어 설정 -------------------- #

# 로봇 SSH 단축 명령어
alias sshbot='ssh ubuntu@192.168.0.4'
# ROS2 Daemon 재시작 단축 명령어
alias rrestart='ros2 daemon stop && sleep 1 && ros2 daemon start'
# ~/.bashrc 업데이트
alias sb='source ~/.bashrc'
# dock, undock 명령어 설정
alias tb2_undock='ros2 action send_goal /robot2/undock irobot_create_msgs/action/Undock "{}"'
alias tb2_dock='ros2 action send_goal /robot2/dock irobot_create_msgs/action/Dock "{}"'
alias tb0_undock='ros2 action send_goal /robot0/undock irobot_create_msgs/action/Undock "{}"'
alias tb0_dock='ros2 action send_goal /robot0/dock irobot_create_msgs/action/Dock "{}"'
# workspace source install/setup.bash
alias si='source install/setup.bash'

# tb2 localization / navigation
# 1단계: localization
alias tb2_nav1='ros2 launch turtlebot4_navigation localization.launch.py namespace:=/robot2 map:=$HOME/rokey_ws/maps/first_map.yaml params_file:=$HOME/rokey_ws/configs/local2.yaml'
# 2단계: Rviz에서 로봇 시각화
alias tb2_nav2='ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/robot2'
# 3단계: navigation stack 실행
alias tb2_nav3='ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/robot2 params_file:=$HOME/rokey_ws/configs/nav2_net2.yaml'
# 4단계: 목표 좌표로 이동
alias tb2_nav4='ros2 run rokey_pjt nav_to_pose --ros-args -r __ns:=/robot2'
