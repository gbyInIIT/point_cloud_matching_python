import vtk
from numpy import random


xmins = [0.0, 0.5]
xmaxs = [0.5, 1.0]
ymins = [0.0, 0.0]
ymaxs = [1.0, 1.0]


class VtkPointCloud:

    def __init__(self, zMin=-10.0, zMax=10.0, maxNumPoints=1e6):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.clearPoints()
        mapper = vtk.vtkPolyDataMapper()
        if vtk.VTK_MAJOR_VERSION <= 5:
            mapper.SetInput(self.vtkPolyData)
        else:
            mapper.SetInputData(self.vtkPolyData)
        # mapper.SetInputData(self.vtkPolyData) THIS IS FOR VTK 6+ see (http://www.vtk.org/Wiki/VTK/VTK_6_Migration/Replacement_of_SetInput)
        mapper.SetColorModeToDefault()
        mapper.SetScalarRange(zMin, zMax)
        mapper.SetScalarVisibility(1)
        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)

    def addPoint(self, point, rgb_color_tuple_3=(0, 255, 0)):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
            self.vtkDepth.InsertNextValue(point[2])
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
            self.vtkColor.InsertNextTuple3(*rgb_color_tuple_3)
        else:
            r = random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:])
            self.vtkColor.InsertNextTuple3(255, 0, 0)


        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def clearPoints(self):
        self.vtkColor = vtk.vtkUnsignedCharArray()
        self.vtkColor.SetNumberOfComponents(3)
        self.vtkColor.SetName("Colors")
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
        self.vtkPolyData.GetPointData().SetActiveScalars('DepthArray')
        self.vtkPolyData.GetPointData().SetScalars(self.vtkColor)
        self.vtkPolyData.Modified()
        if vtk.VTK_MAJOR_VERSION <= 5:
            self.vtkPolyData.Update()

    def display_self_mirror(self):
        # Renderer
        renderer = vtk.vtkRenderer()
        renderer.AddActor(self.vtkActor)
        renderer.SetBackground(1, 1, 1)
        renderer.ResetCamera()
        renderer.SetViewport(xmins[0], ymins[0], xmaxs[0], ymaxs[0])

        rendererRight = vtk.vtkRenderer()
        rendererRight.AddActor(self.vtkActor)
        rendererRight.SetBackground(1, 1, 1)
        rendererRight.ResetCamera()
        rendererRight.SetViewport(xmins[1], ymins[1], xmaxs[1], ymaxs[1])
        rendererRight.SetActiveCamera(renderer.GetActiveCamera())
        # Render Window
        renderWindow = vtk.vtkRenderWindow()
        renderWindow.AddRenderer(renderer)
        renderWindow.AddRenderer(rendererRight)
        # Interactor
        renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        renderWindowInteractor.SetRenderWindow(renderWindow)
        # Begin Interaction
        renderWindow.Render()
        renderWindowInteractor.Start()
        renderWindow.Finalize()
        renderWindowInteractor.TerminateApp()
        del renderWindow, renderWindowInteractor


def main():
    pointCloud = VtkPointCloud()
    for k in xrange(1000):
        point = 20*(random.rand(3)-0.5)
        pointCloud.addPoint(point)
    pointCloud.addPoint([0,0,0])
    pointCloud.addPoint([0,0,0])
    pointCloud.addPoint([0,0,0])
    pointCloud.addPoint([0,0,0])

    pointCloud.display_self_mirror()

if __name__ == '__main__':
    main()
