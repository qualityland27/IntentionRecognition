# Dockerfile for intention_recognition


############################
#        Install ROS       #
############################
#FROM tiryoh/ros-melodic-desktop
FROM osrf/ros:melodic-desktop-full

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
#SHELL ["/bin/bash", "-c"] 

# IP of MASTER and from Host of this container needed 
RUN echo 'export ROS_MASTER_URI=http://129.13.91.44:11311' >> /root/.bashrc
RUN echo 'export ROS_IP=129.13.90.232' >> /root/.bashrc

RUN echo 'source /opt/ros/melodic/setup.bash' >> /root/.bashrc

############################
#     Install MediaPipe    #
############################
WORKDIR /io
WORKDIR /mediapipe

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        gcc-8 g++-8 \
        ca-certificates \
        curl \
        ffmpeg \
        git \
        wget \
        unzip \
#        python3 \
        python3-dev \
        python3-opencv \
        python3-pip \
        libopencv-core-dev \
        libopencv-highgui-dev \
        libopencv-imgproc-dev \
        libopencv-video-dev \
        libopencv-calib3d-dev \
        libopencv-features2d-dev \
        software-properties-common && \
    add-apt-repository -y ppa:openjdk-r/ppa && \
    apt-get update && apt-get install -y openjdk-8-jdk && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 100 --slave /usr/bin/g++ g++ /usr/bin/g++-8
RUN pip3 install --upgrade setuptools
RUN pip3 install wheel
RUN pip3 install future
RUN pip3 install six==1.14.0
RUN pip3 install tensorflow==1.14.0
RUN pip3 install tf_slim

# RUN ln -s /usr/bin/python3 /usr/bin/python

# Install bazel
ARG BAZEL_VERSION=3.7.2
RUN mkdir /bazel && \
    wget --no-check-certificate -O /bazel/installer.sh "https://github.com/bazelbuild/bazel/releases/download/${BAZEL_VERSION}/b\
azel-${BAZEL_VERSION}-installer-linux-x86_64.sh" && \
    wget --no-check-certificate -O  /bazel/LICENSE.txt "https://raw.githubusercontent.com/bazelbuild/bazel/master/LICENSE" && \
    chmod +x /bazel/installer.sh && \
    /bazel/installer.sh  && \
    rm -f /bazel/installer.sh

#COPY . /mediapipe/

RUN pip3 install --upgrade pip
RUN pip3 install mediapipe


############################
#       Install Other      #
############################
RUN apt-get update && apt-get install -y \
        net-tools \
        vim \
#        python \
        python-dev \
        python-opencv \
        python-pip \
#        cmake \ 
#        python-catkin-pkg \ 
#        python-empy \
#        python-nose \ 
#        python-setuptools \ 
#        libgtest-dev \
#        build-essential \
        ros-melodic-roslib \
        ros-melodic-rospy \
        ros-melodic-libfranka \
        ros-melodic-franka-ros && \
    add-apt-repository -y ppa:openjdk-r/ppa && \
    apt-get update && apt-get install -y openjdk-8-jdk && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN pip3 install pandas
RUN pip3 install numpy
RUN pip3 install pyrealsense2
RUN pip3 install matplotlib
RUN pip3 install scikit-image
RUN pip3 install Pillow
RUN pip3 install colorlog
RUN pip3 install scikit-learn
RUN pip3 install scipy
RUN pip3 install joblib
RUN pip3 install statsmodels
RUN pip3 install numba
RUN pip3 install pathlib
RUN pip3 install setuptools
RUN pip3 install python-dotenv

RUN pip install --upgrade pip
RUN pip install pyyaml
RUN pip install rospkg

RUN sudo usermod -a -G video root

RUN apt update -y && apt install -y \
	v4l-utils \
	libsm6 \
	libxext6 \
	libxrender-dev \
	ros-melodic-catkin && \
    add-apt-repository -y ppa:openjdk-r/ppa && \
    apt-get update && apt-get install -y openjdk-8-jdk && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY . /system/

RUN cd /system/intention_reaction_ws/ && rm -r devel/ && rm -r build/ && rm -r logs/ && rm -r .catkin_tools/

RUN cd /system/intention_reaction_ws/ && source /opt/ros/melodic/setup.bash && catkin_make

RUN echo 'source /system/intention_reaction_ws/devel/setup.bash' >> /root/.bashrc

WORKDIR /system/intention_recognition/

CMD ["python3", "make-detection.py"]

