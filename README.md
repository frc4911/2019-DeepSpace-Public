Team 4911's 2019 Codebase
===

# Quick Commands
## Assemble
`./gradlew assemble`  
Build will compile and get the code ready without deploying it without running all automated tests.

## Build
`./gradlew build`  
Build will compile and get the code ready without deploying it. It will also run all automated tests, which is great for testing your code before it ever gets run on a robot (which also means you can build whenever)

## Deploy
`./gradlew deploy`  
Deploying will build the robot code (as above), and deploy it to the robot. You have to be connected to the robot for this to work. Just keep in mind that deploying _does not run any automated tests_.

# Projects
We break our code up into robot code and cheesylib library. 

## [src](src)
2019 robot code and utilities.

## [cheesylib](cheesylib)
cheesylib was written by Team 254 for the 2018 season.
