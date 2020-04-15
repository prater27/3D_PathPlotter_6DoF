#include "resultsreader.h"
#include <vtkInteractorStyleUnicam.h>
#include <vtkInteractorStyleJoystickCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <chrono>
#include <vtkMultiThreader.h>

#include <iostream>

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSphereSource.h>
#include <vtkElevationFilter.h>
#include <vtkVectorText.h>
#include <vtkCommand.h>

unsigned int processedPointsCounter=1; //global - The first frame is created manually, since the animation does not
//start until Return key is pressed, therefore the first point is "pre-processed" prior to the callback system.
bool animStart=false; //global
std::chrono::time_point<std::chrono::steady_clock> startTime;

void KeypressCallbackFunction ( vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* vtkNotUsed(clientData), void* vtkNotUsed(callData) )
{
    vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);

    std::string ret = "Return";
    std::string pushed(iren->GetKeySym());
  // Handle an arrow key
  if(pushed == ret && animStart==false)
  {
        startTime = std::chrono::steady_clock::now();
        iren->CreateRepeatingTimer(50);
        animStart=true;
  }
}

void updateSpline(const vtkSmartPointer<vtkParametricSpline>& spline, const vtkSmartPointer<vtkParametricFunctionSource>& functionSource,
                  const vtkSmartPointer<vtkPoints>& points){
    spline->SetNumberOfPoints(points->GetNumberOfPoints());
    spline->SetPoints(points);

    functionSource->SetParametricFunction(spline);
    functionSource->SetUResolution(50 * points->GetNumberOfPoints());
    functionSource->SetVResolution(50 * points->GetNumberOfPoints());
    functionSource->SetWResolution(50 * points->GetNumberOfPoints());
    functionSource->Update();
}

void updateRocketCoordSystem(const double (&pos)[3], const double (&rot)[4],
                             const vtkSmartPointer<vtkAxesActor>& actorAxes, const vtkSmartPointer<vtkTransform>& transform){
    vtkQuaterniond quat(rot[0], rot[1], rot[2], rot[3]);
    transform->Identity();//We give the whole time absolute position and rotation
    transform->Translate(pos[0], pos[1], pos[2]);
    //Rotation in degrees    //// https://vtk.org/doc/nightly/html/classvtkQuaternion.html Roation with quaternions possible (check)
    //In reality the transformation will be done with the passed quaternion
    transform->RotateX(rot[0]);
    transform->RotateY(rot[1]);
    transform->RotateZ(rot[2]);
    actorAxes->SetUserTransform(transform);
}

void updatePoints(const double (&pos)[3], const vtkSmartPointer<vtkPoints>& points,
                  const vtkSmartPointer<vtkPolyData>& polyDataPoints, const vtkSmartPointer<vtkCellArray>& verts)
 {
    //Create Points polydata
    vtkIdType id = points->InsertNextPoint(pos);
    verts->InsertNextCell(1, &id);
    polyDataPoints->SetPoints(points);
    polyDataPoints->SetVerts(verts);
}

struct UpdateParametersArguments{
    vtkSmartPointer<vtkParametricSpline> spline;
    vtkSmartPointer<vtkParametricFunctionSource> functionSource;
    vtkSmartPointer<vtkPoints> points;
    std::vector<ResultsLine> results;

    vtkSmartPointer<vtkPolyData> polyDataPoints;
    vtkSmartPointer<vtkCellArray> verts;
    vtkSmartPointer<vtkAxesActor> actorAxes;
    vtkSmartPointer<vtkTransform> transform;

   UpdateParametersArguments():
        spline(),
        functionSource(),
        points(),
        results(),
        polyDataPoints(),
        verts(),
        actorAxes(),
        transform()
        {}

   UpdateParametersArguments(const vtkSmartPointer<vtkParametricSpline>& spline_,
                             const vtkSmartPointer<vtkParametricFunctionSource>& functionSource_,
                             const vtkSmartPointer<vtkPoints>& points_,
                             const std::vector<ResultsLine>& results_,
                             const vtkSmartPointer<vtkPolyData>& polyDataPoints_,
                             const vtkSmartPointer<vtkCellArray>& verts_,
                             const vtkSmartPointer<vtkAxesActor>& actorAxes_,
                             const vtkSmartPointer<vtkTransform>& transform_):
        spline(spline_),
        functionSource(functionSource_),
        points(points_),
        results(results_),
        polyDataPoints(polyDataPoints_),
        verts(verts_),
        actorAxes(actorAxes_),
        transform(transform_)
        {}

   std::vector<ResultsLine>* getPtrToResults(){return &results;};
};

void updateData(void* argums)
{
  UpdateParametersArguments* arguments = static_cast<UpdateParametersArguments*>(argums);
  updatePoints(arguments->results[processedPointsCounter].pos, arguments->points, arguments->polyDataPoints, arguments->verts);
  updateSpline(arguments->spline, arguments->functionSource, arguments->points);

  updateRocketCoordSystem(arguments->results[processedPointsCounter].pos, arguments->results[processedPointsCounter].rot, arguments->actorAxes, arguments->transform);
}

void TimerCallbackFunction ( vtkObject* caller, long unsigned int vtkNotUsed(eventId), void* clientData, void* vtkNotUsed(callData) )
{
    UpdateParametersArguments* argums = static_cast<UpdateParametersArguments*>(clientData);

    vtkRenderWindowInteractor *iren = static_cast<vtkRenderWindowInteractor*>(caller);

    //La mejor forma de conseguir resultados similares al tiempo real es con este if
  //  if((argums->getPtrToResults()->size()>processedPointsCounter)) //&&
       // (1000*(argums->results[processedPointsCounter].t)<(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - startTime).count())))
//    {
    //        processedPointsCounter++; fuera del if
    //y el timer creado al principio en repeticion cada pocos segundos


    if((argums->getPtrToResults()->size()>processedPointsCounter))
    {
        updateData(clientData); //Here the processedPointsCounter increases, hence the current point being processed is at "processedPointsCounter"
        iren->Render();
       //This code can be used for creating one shot timer corresponding to when should be the next point drawn
        //according to its time
        // unsigned long nextTimeStep= ceil(1000*(*(argums->results[processedPointsCounter+1].getPointerToTime())-
        //                                                                *(argums->results[processedPointsCounter].getPointerToTime())));
        //iren->CreateOneShotTimer(nextTimeStep);
        processedPointsCounter++;
    }
}


int main(int argc, char* argv[])
{
  // Verify input arguments
  if ( argc != 2 )
  {
    std::cout << "Usage: " << argv[0]
              << " Filename(.xyz)" << std::endl;
    return EXIT_FAILURE;
  }

// Read the file
  ResultsReader reader;
  std::vector<ResultsLine> results;
  //Reads the data, and exits in case there were any problem while reading points
  if (reader.read(std::string(argv[1]), results)==0){return 0;};

//Colors
  vtkSmartPointer<vtkNamedColors> colors = vtkSmartPointer<vtkNamedColors>::New();
  vtkColor3d backgroundColor = colors->GetColor3d("SlateGray");
  vtkColor3d legendBackgroundColor = colors->GetColor3d("Black");

//STATIC ELEMENTS

//Plane
  // Set the background color.
  // Create a plane
    double planeSize = 200.0;
    vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetCenter(0.0, 0.0, 0.0);
    planeSource->SetPoint1(planeSize/2, 0.0, 0.0);
    planeSource->SetPoint2(0.0, planeSize/2, 0.0);
    planeSource->Update();

    vtkPolyData* plane = planeSource->GetOutput();

    // Create a mapper and actor for the plane
    vtkSmartPointer<vtkPolyDataMapper> mapperPlane = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapperPlane->SetInputData(plane);

    vtkSmartPointer<vtkActor> actorPlane = vtkSmartPointer<vtkActor>::New();
    actorPlane->SetMapper(mapperPlane);
    actorPlane->GetProperty()->SetOpacity(0.5);
    //actorPlane->GetProperty()->SetColor(colors->GetColor3d("Cyan").GetData());

//Coordinate System Origin
    //Create actor coordinate system axes
    vtkSmartPointer<vtkAxesActor> actorAxesOrigin = vtkSmartPointer<vtkAxesActor>::New();
    //Change the size of the axes, change label size, change label text, change label color...// axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(1,0,0);
    actorAxesOrigin->SetTotalLength(planeSize/3, planeSize/3, planeSize/3);
    actorAxesOrigin->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(1);
    actorAxesOrigin->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(1);
    actorAxesOrigin->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(1);
    actorAxesOrigin->SetXAxisLabelText("");
    actorAxesOrigin->SetYAxisLabelText("");
    actorAxesOrigin->SetZAxisLabelText("");


//DYNAMIC ELEMENTS

//Points
    // Allocate objects to hold points and vertex cells.
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> verts = vtkSmartPointer<vtkCellArray>::New();
    //Allocate memory for the polydata formed from the previous points and cells
    vtkSmartPointer<vtkPolyData> polyDataPoints = vtkSmartPointer<vtkPolyData>::New();
    //Allocate memory for the  Mapper for the Points PolyData
    vtkSmartPointer<vtkPolyDataMapper> mapperPoints = vtkSmartPointer<vtkPolyDataMapper>::New();
    //Allocate memory for the Points actor
    vtkSmartPointer<vtkActor> actorPoints = vtkSmartPointer<vtkActor>::New();


//Rocket Coordinate System
    //Allocate memory for the rockets coordinate system actor
    vtkSmartPointer<vtkAxesActor> actorAxes = vtkSmartPointer<vtkAxesActor>::New();
    //Allocate memory for the rockets coordinate system transformation
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

//Spline
    //Allocate memory for all the splines and related functions
    vtkSmartPointer<vtkKochanekSpline> xSpline = vtkSmartPointer<vtkKochanekSpline>::New();
    vtkSmartPointer<vtkKochanekSpline> ySpline = vtkSmartPointer<vtkKochanekSpline>::New();
    vtkSmartPointer<vtkKochanekSpline> zSpline = vtkSmartPointer<vtkKochanekSpline>::New();
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
    vtkSmartPointer<vtkParametricFunctionSource> functionSource = vtkSmartPointer<vtkParametricFunctionSource>::New();
    //Allocates memory for the Spline's actor and mapper
    vtkSmartPointer<vtkPolyDataMapper> mapperSpline = vtkSmartPointer<vtkPolyDataMapper>::New();
    vtkSmartPointer<vtkActor> actorSpline = vtkSmartPointer<vtkActor>::New();

//Renderer and windows
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();

//VISUALIZATION AND FIRST FRAME CREATION
//Points
    vtkIdType id = points->InsertNextPoint(results.begin()->pos);
    verts->InsertNextCell(1, &id);
    polyDataPoints->SetPoints(points);
    polyDataPoints->SetVerts(verts);

    //Create Points PolyData Mapper
    mapperPoints->SetInputData(polyDataPoints);
    //Create Points PolyData actor
    actorPoints->SetMapper(mapperPoints);
    actorPoints->GetProperty()->SetPointSize(2);

//Rocket coordinate system
  //The axes of the rocket's coordinate system are positioned with a user transform. Changes the size of the axes
    vtkQuaterniond quat(results[0].rot[0], results[0].rot[1], results[0].rot[2], results[0].rot[3]);
    transform->Translate(results[0].pos[0], results[0].pos[1], results[0].pos[2]);
    //Rotation in degrees    //// https://vtk.org/doc/nightly/html/classvtkQuaternion.html Roation with quaternions possible (check)
    //In reality the transformation will be done with the previously created quaternion quat
    transform->RotateX(16.0);
    transform->RotateY(160.0);
    transform->RotateZ(45.0);
    actorAxes->SetUserTransform(transform);

    actorAxes->SetTotalLength(planeSize/5, planeSize/5, planeSize/5);
    actorAxes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(1);
    actorAxes->SetXAxisLabelText("");
    actorAxes->SetYAxisLabelText("");
    actorAxes->SetZAxisLabelText("");

//Spline - In the first frame there is not passed path, only initial position
  spline->SetXSpline(xSpline);
  spline->SetYSpline(ySpline);
  spline->SetZSpline(zSpline);
  spline->SetPoints(points);

  functionSource->SetParametricFunction(spline);
  functionSource->SetUResolution(50 * points->GetNumberOfPoints());
  functionSource->SetVResolution(50 * points->GetNumberOfPoints());
  functionSource->SetWResolution(50 * points->GetNumberOfPoints());
  functionSource->Update();

  mapperSpline->SetInputConnection(functionSource->GetOutputPort());
  actorSpline->SetMapper(mapperSpline);
  actorSpline->GetProperty()->SetColor(colors->GetColor3d("DarkSlateGrey").GetData());
  actorSpline->GetProperty()->SetLineWidth(3.0);


  //Creates the initial structures that will be passed to the function called by the timer callback,
  //containing the objects to be updated
  UpdateParametersArguments updateParametersArguments(spline, functionSource, points, results, polyDataPoints,
                                                      verts, actorAxes, transform);
  void* arguments = static_cast<void*>(&updateParametersArguments);

//Render
  //Add actors
  renderer->AddActor(actorPoints);
  renderer->AddActor(actorAxes);
  renderer->AddActor(actorSpline);
  renderer->AddActor(actorPlane);
  renderer->AddActor(actorAxesOrigin);
  //Set Background color
  renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());
  //Sets the window rendering process with the set renderer
  renderWindow->AddRenderer(renderer);
  //Sets a windown interactor with the rendering window
  renderWindowInteractor->SetRenderWindow(renderWindow);

  //Callback for the keypress event - Press enter and the animation starts
  vtkSmartPointer<vtkCallbackCommand> keypressCallback = vtkSmartPointer<vtkCallbackCommand>::New();
  keypressCallback->SetCallback ( KeypressCallbackFunction );
  renderWindowInteractor->AddObserver ( vtkCommand::KeyPressEvent, keypressCallback );

  //Add camera interactor
  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style = vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  renderWindowInteractor->SetInteractorStyle(style);
  style->SetCurrentRenderer(renderer);

  //Initialize must be called prior to creating timer events.
  renderWindowInteractor->Initialize();
  vtkSmartPointer<vtkCallbackCommand> timerCallback = vtkSmartPointer<vtkCallbackCommand>::New();
  timerCallback->SetCallback(TimerCallbackFunction);
  timerCallback->SetClientData(arguments);
  renderWindowInteractor->AddObserver ( vtkCommand::TimerEvent, timerCallback );
  renderWindowInteractor->AddObserver ( vtkCommand::LeftButtonReleaseEvent, timerCallback );

  //Renders the window
  renderWindow->Render();
  //Starts the window interactor
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
