The document contains the source code of each algorithm created for the Diploma Thesis

The individual algorithms are placed in separate folders.
Since there are subtle differences in data display and processing, 
the algorithms are divided into 3 parts (SEF into 2) by separating them with the suffix :
							
	_createdData : The algorithms process the selected created data e.g. ("imgs/pointCloud4.png")
	
	_oneMeasurement : The algorithms process one of 107 experimental measurements e.g. [63]

	_multipleMeasurements : Algorithms process a set of multiple connected experimental measurements e.g. [0:20] 

The data_python partition contains the measurement data files.
The imgs partition contains the files with the generated data.

The SplitandMerge, Ransac and Hough algorithms process the bitmap image they convert as input data.
This is to demonstrate their ability to process such input data and their ability to process a disordered array of points. 
In the measured data part, they process already input data in the form of an array of points.


Thesis 2021
Marek Šooš