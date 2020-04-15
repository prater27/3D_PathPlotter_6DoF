#include <iostream>
#include <fstream>
#include <vector>
#include <fstream>
#include <sstream>
#include <cstdio>
#include <vector>

#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkQuaternion.h>
#include <vtkTransform.h>
#include "vtkPoints.h"
#include "vtkCellArray.h"
#include "vtkObjectFactory.h"
#include "vtkPoints.h"
#include "vtkPolyData.h"
#include "vtkSmartPointer.h"

//CoordinateSystem
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkCaptionActor2D.h>
#include <vtkNamedColors.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>

//Spline
#include <vtkKochanekSpline.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkProperty.h>
#include <vtkGlyph3DMapper.h>
#include <vtkNamedColors.h>

//Plane
#include <vtkPlaneSource.h>
#include <vtkLegendBoxActor.h>
#include <vtkTransformPolyDataFilter.h>

//Animation
#include <vtkProgrammableFilter.h>
#include <vtkCommand.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCallbackCommand.h>

#include <string>
#include <array>

#include "vtkPolyDataAlgorithm.h"

struct ResultsLine{
    double t;
    double pos[3];
    double rot[4];

    double* getPointerToTime(){ return &t;};
};

class ResultsReader{

public:
    int read(const std::string& fileName, std::vector<ResultsLine>& results)
    {
        std::ifstream file(fileName.c_str());

        std::string str;
        std::string fileLine;
        unsigned long linesCounter=0;

        while(std::getline(file, fileLine))
        {
            int lineElementsCounter=0;
                std::stringstream lineStream(fileLine);
                ResultsLine currentLineResults;

                double value;
                while(lineStream >> value)
                {
                    if(lineElementsCounter==0){
                        currentLineResults.t=value;
                    }
                    else if(lineElementsCounter<=3){
                        currentLineResults.pos[lineElementsCounter-1]=value;
                    }
                    else if(lineElementsCounter<=7){
                        currentLineResults.rot[lineElementsCounter-4]=value;
                    }
                    lineElementsCounter++;
                }
                if(lineElementsCounter!=8){
                    std::cout << "The line " << linesCounter << "contains at least one wrong data!\n";
                    return 0;
                    break;
                }
                else{
                    results.push_back(currentLineResults);
                }
            linesCounter++;
        }
        //Checks that the file was correctly read
        if((results.size() != linesCounter) || linesCounter<2)
        {
            std::cout << "There is an error in your input file and not all the points were well read, or there is too few points.\n"
                         "Please check the file and try again!\n\n";
            return 0;
        }
        else{
            return 1;
        }
    }
};

