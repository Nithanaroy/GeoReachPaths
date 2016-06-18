# GeoReachPaths
The aim of the project is to perform a joint social and spatial search on a socio-spatial graph and compare the performance benefits.

## Detailed Challenge Statement
Find K shortest paths from a vertex to a region in the world. In other words, given a yelp graph which has social connections among users and also check-ins/reviews at multiple restaurents, we want to find K shortest paths from a user, say Alice, to a region, say Los Angeles, USA. The result would be a list of paths starting at Alice and ending at restaurants in Los Angeles, USA. Here path is an ordered list of vertices which is a valid route in the graph.

## Folder Structure
	Common.py => has scripts that are used by various modules
	DataPrep.py => has scripts which are used for data preparation
	GeoReachPaths.py => the main file which has different approaches to the problem
	LICENSE
	Naive.py => has various naive/simple/straight-forward approaches to the problem
	requirements.txt
	tests
		GeoReachRunners.py => has scripts to run all the approaches with different parameter combinations and also pretty print the output for generating charts
		Test_GeoReachPaths.py => has unit testing code for critical pices of the code
		__init__.py
		results
			__init__.py
			
## How to run
### Prepare the data
  - Download the Yelp data from https://www.yelp.com/dataset_challenge
  - We will use user.json, business.json and review.json to create our graph. To create a **weighted** graph use the script `DataPrep.add_weight_to_yelp`

### Test the scripts
  - Go through public methods in `GeoReachPaths`
  - Run the tests in `TestGeoReachPaths`
  
### Run the scripts
  - `GeoReachPaths` has multiple approaches and comments under each public method give a basic idea. For complete details we will publish a PDF soon.
  - `GeoReachRunners.yelp_runner` generats a JSON which runs all approaches with different input paramaters
  - `GeoReachRunner.pretty_print` generates different tables which can be fed into a charting system to create plots
  
For any queries please contact npasumar [at] asu [dot] edu
