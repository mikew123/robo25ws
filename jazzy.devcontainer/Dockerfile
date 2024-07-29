FROM arm64v8/ros:jazzy-perception-noble

# Create a non-root user
ARG USERNAME=robo25
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd -o --gid $USER_GID $USERNAME \
  && useradd -o -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config \
  && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
  && apt-get install -y \
  nano \
  vim \
  minicom \
  curl \
  gnupg2 \
  python3-full \
  lsb-release \
  bash-completion \
  python3-argcomplete \
  locales \
  python3-pip \
  && apt-get clean \
  && rm -rf /var/lib/apt/lists/*

  ###   linux-modules-extra-raspi \


RUN locale-gen en_US en_US.UTF-8 
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
RUN usermod -aG dialout $USERNAME 
#RUN python3 -m pip install pyserial
#RUN pip3 install pyserial
#RUN apt-get install -y python3-serial


# Set the locale
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]
