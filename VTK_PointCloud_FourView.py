import vtk
from numpy import random
from VTK_PointCloud import VtkPointCloud

# xmins = [0.0, 0.5, 0.0, 0.5]
# xmaxs = [0.5, 1.0, 0.5, 1.0]
# ymins = [0.0, 0.0, 0.5, 0.5]
# ymaxs = [0.5, 0.5, 1.0, 1.0]
xmins = [0.0, 0.25, 0.5, 0.75]
xmaxs = [0.25, 0.5, 0.75, 1.0]
ymins = [0.0, 0.0, 0.0, 0.0]
ymaxs = [1.0, 1.0, 1.0, 1.0]


class VtkFourViewDisplay:
    def __init__(self, thirdPointCloud, fourthPointCloud, firstPointCloud, secondPointCloud, back_ground_rgb_tuple_3=(0, 0, 0), point_size=1):
        # Renderer
        self.rendererLeftUpper = vtk.vtkRenderer()
        thirdPointCloud.vtkActor.GetProperty().SetPointSize(point_size)
        self.rendererLeftUpper.AddActor(thirdPointCloud.vtkActor)
        self.rendererLeftUpper.SetBackground(*back_ground_rgb_tuple_3)
        self.rendererLeftUpper.ResetCamera()
        self.rendererLeftUpper.SetViewport(xmins[2], ymins[2], xmaxs[2], ymaxs[2])
        cameraLeftUpper = vtk.vtkCamera()
        cameraLeftUpper.SetPosition(0, 0, 1)
        cameraLeftUpper.SetFocalPoint(0, 0, 0)
        self.rendererLeftUpper.SetActiveCamera(cameraLeftUpper)

        self.rendererRightUpper = vtk.vtkRenderer()
        fourthPointCloud.vtkActor.GetProperty().SetPointSize(point_size)
        self.rendererRightUpper.AddActor(fourthPointCloud.vtkActor)
        self.rendererRightUpper.SetBackground(*back_ground_rgb_tuple_3)
        self.rendererRightUpper.SetViewport(xmins[3], ymins[3], xmaxs[3], ymaxs[3])
        self.rendererRightUpper.SetActiveCamera(self.rendererLeftUpper.GetActiveCamera())
        
        self.rendererLeftLower = vtk.vtkRenderer()
        firstPointCloud.vtkActor.GetProperty().SetPointSize(point_size)
        self.rendererLeftLower.AddActor(firstPointCloud.vtkActor)
        self.rendererLeftLower.SetBackground(*back_ground_rgb_tuple_3)
        self.rendererLeftLower.SetViewport(xmins[0], ymins[0], xmaxs[0], ymaxs[0])
        self.rendererLeftLower.SetActiveCamera(self.rendererLeftUpper.GetActiveCamera())

        self.rendererRightLower = vtk.vtkRenderer()
        secondPointCloud.vtkActor.GetProperty().SetPointSize(point_size)
        self.rendererRightLower.AddActor(secondPointCloud.vtkActor)
        self.rendererRightLower.SetBackground(*back_ground_rgb_tuple_3)
        self.rendererRightLower.SetViewport(xmins[1], ymins[1], xmaxs[1], ymaxs[1])
        self.rendererRightLower.SetActiveCamera(self.rendererLeftUpper.GetActiveCamera())

        # Render Window
        self.renderWindow = vtk.vtkRenderWindow()
        self.renderWindow.SetSize(1600, 800)
        self.renderWindow.AddRenderer(self.rendererLeftUpper)
        self.renderWindow.AddRenderer(self.rendererRightUpper)
        self.renderWindow.AddRenderer(self.rendererLeftLower)
        self.renderWindow.AddRenderer(self.rendererRightLower)
        # Interactor
        self.renderWindowInteractor = vtk.vtkRenderWindowInteractor()
        self.renderWindowInteractor.SetRenderWindow(self.renderWindow)

    def display(self):
        # Begin Interaction
        self.renderWindow.Render()
        self.renderWindowInteractor.Start()
        self.renderWindow.Finalize()
        self.renderWindowInteractor.TerminateApp()
        del self.renderWindow, self.renderWindowInteractor


def main():
    pointCloudLeft = VtkPointCloud()
    for k in xrange(1000):
        point = 20*(random.rand(3)-0.5)
        pointCloudLeft.addPoint(point)
    pointCloudLeft.addPoint([0,0,0], (255, 0, 0))

    pointCloudRight = VtkPointCloud()
    for k in xrange(1000):
        point = 20*(random.rand(3)-0.5)
        pointCloudRight.addPoint(point, (255, 0, 0))
    pointCloudRight.addPoint([0,0,0])

    pointCloudLeftLower = VtkPointCloud()
    for k in xrange(1000):
        point = 20*(random.rand(3)-0.5)
        pointCloudLeftLower.addPoint(point, (0, 0, 255))
    pointCloudLeftLower.addPoint([0,0,0])

    pointCloudRightLower = VtkPointCloud()
    for k in xrange(1000):
        point = 20*(random.rand(3)-0.5)
        pointCloudRightLower.addPoint(point, (255, 0, 255))
    pointCloudRightLower.addPoint([0,0,0])

    four_view = VtkFourViewDisplay(thirdPointCloud=pointCloudLeft,
                                   fourthPointCloud=pointCloudRight,
                                   firstPointCloud=pointCloudLeftLower,
                                   secondPointCloud=pointCloudRightLower)
    four_view.display()


if __name__ == '__main__':
    main()
