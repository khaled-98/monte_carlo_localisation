# Monte Carlo Localisation

A ROS implementation of the Monte Carlo Localisation (MCL) and some of its variations as described in [Probablistic Robotics](http://www.probabilistic-robotics.org/).

In addition to the "vanilla" MCL algorithm, this code also implementes:

* Augmented Resampling MCL
* KLD MCL

> :warning: **_NOTE:_**  Basic testing of this implementation shows that it works; however, it was written primarily as a learning excercise. As a result, there are several areas where the code can be improved.

<p align="center">
  <img width="500" height="505" src="demo.gif">
</p>


## Background

My main goal before writing this code was to gain an intuitive understanding of MCL and its well known variants. I wanted to play around with an implementation of the algorithm and modify it to build this intuition.

Initially I was planning to use an existing implementation and I have found several ones out there (most notably [ROS' AMCL](http://wiki.ros.org/amcl)). However, most of the implemenations that I came across were overly complicated so I decided to write my own.