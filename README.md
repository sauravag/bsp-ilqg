bsp-ilqg
=============

Belief Space Motion Planning Using iLQG. Builds on top of iLQG Matlab implementation by Yuval Tassa and
the paper "Motion Planning under Uncertainty using Iterative Local Optimization in Belief Space", Van den berg et al., 
International Journal of Robotics Research, 2012

How To
=============

There are demo files provided in the main directory that you can straight away run and see planning scenarios.
Inside each of these demo files you can change the map to be loaded (i.e., the planning scenario) and you can change
the start and goal state.

A modular appraoch has been taken to implement this code, all motion models for robot and observation models for
sensing are based on base classes for each, enabling you to write your own models. Simple collision checking for
a circular (disk) like robot is provided along with a bunch of 2D maps. To implement your own planning scenario, write your 
own motion model and observation model. Then copy one of the demo files and in the part of code where the models are initialized 
replace with your own models.

License
=============

Copyright (c) 2017, Saurav Agarwal
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the distribution
    * Neither the name of the Texas A&M University nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
