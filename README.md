# Northeastern’s NASA Valkyrie Humanoid Robot Dataset

Below are instructions on how to access and visualize Northeastern's NASA Valkyrie Humanoid Robot Dataset, as well as instructions on how to generate your own data using the simulated Valkyrie robot and visualizing the data in the same manor.

## The Dataset

### Download the Dataset
 [Dataset](http://hdl.handle.net/2047/D20249932) (435 MB)

### Visualizing the Dataset
 Instructions for use:
 
 1. Download the dataset
 2. Import one of the Matlab data files (found in the dataset) into your workspace corresponding to the trial you wish to view
 3. Run plot_data (found in the Matlab directory in this repository) from the command line or the "Run" button

### Dataset Video Example
 [![Dataset Youtube Video](https://img.youtube.com/vi/H5slSraVXas/0.jpg)](https://youtu.be/H5slSraVXas)


---------------------------------------


## Running and Logging Your Own Experiments

To run the Valkyrie simulator and use IHMC's logger (which is capable of logging all internal robot state information), it is highly recommended that you use two computers connected with a wired connection.

### Installation 
#### Installing the Valkyrie Simulator
> Instructions to come...

#### Installing and Setting up IHMC's Logger
> More Instructions to come...

[IHMC Logger (0.8.2)](https://bintray.com/ihmcrobotics/distributions/IHMCLogger/0.8.2)


### Moving the robot
Scripts used to generate the dataset can be found in the ROS directory and can be also used as an example of controlling the Valkyrie robot.


### Processing the data
Scripts to process the logged data are in the Matlab directory.

#### Prereqs for Dataset Processing
* [Kabsch.m](http://www.mathworks.com/matlabcentral/fileexchange/25746-kabsch-algorithm)
  * Helpful reference: [Kabsch Algorithm Wikipedia](https://en.wikipedia.org/wiki/Kabsch_algorithm)
* [absor.m](https://www.mathworks.com/matlabcentral/fileexchange/26186-absolute-orientation-horn-s-method)
