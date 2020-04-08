# Rapidly-Exploring-Random-Trees
My C++ implementations of RRT, RRT* and anytime RRT (robotics algorithms used in motion planning). 

## Requirements

C++ version 11+ and SFML (Simple and Fast Multimedia Library) is required to run the codes. 

## Installation 

<details><summary><b>For Linux</b></summary>

1. Install the SFML lib:

    ```sh
    $ sudo apt-get install libsfml-dev
    ```
    
</details>

## How to Run 

```
 g++ -std=c++11 -c geometry.h rrt.cpp 
```

```
g++ rrt.o -o sfml-app -lsfml-graphics -lsfml-window -lsfml-system
```

```
./sfml-app 
```

## Usage 

```
NOTE:
Height of screen: 600 pixels. Width of screeen: 800 pixels.
Maximum distance by which algorithm jumps from one point to another: 32 units
If you would like to change of any of these, please make modifications in code
Please provide your inputs keeping this in mind. 

Which type of RRT would you like to watch? 1 for RRT, 2 for RRT*, 3 for Anytime RRT
3
Input co-ordinates of starting and ending point respectively in this format X1 Y1 X2 Y2
100 70
600 400
How many obstacles?
2
How many points in 1th polygon?
4
Input co-ordinates of 1th polygon in clockwise order
200 480
200 100
250 100
250 480
How many points in 2th polygon?
5
Input co-ordinates of 2th polygon in clockwise order
400 0
300 100
350 250
450 250
500 100
```
