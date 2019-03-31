#!/usr/bin/env python

import vtk



class vtkTimerCallback():
    def __init__(self, steps, actor, iren):
        self.timer_count = 0
        self.steps = steps
        self.actor = actor
        self.iren = iren
        self.timerId = None

    def execute(self, obj, event):
        step = 0
        while step < self.steps:
            print(self.timer_count)
            if step < self.steps/3:
                self.actor.RotateX(1/4)
            elif step > self.steps/3 and step <self.steps*2/3:
                self.actor.RotateY(1/4)
            else:
                self.actor.RotateZ(1/4)
            iren = obj
            iren.GetRenderWindow().Render()
            self.timer_count += 1
            step += 1
        if self.timerId:
            iren.DestroyTimer(self.timerId)



def main():
    colors = vtk.vtkNamedColors()

    # create a Sphere
    sphereSource = vtk.vtkSphereSource()
    sphereSource.SetCenter(0.0, 0.0, 0.0)
    sphereSource.SetRadius(0.5)

    # create a mapper
    sphereMapper = vtk.vtkPolyDataMapper()
    sphereMapper.SetInputConnection(sphereSource.GetOutputPort())

    # create an actor
    sphereActor = vtk.vtkActor()
    sphereActor.SetMapper(sphereMapper)

    # a renderer and render window
    renderer = vtk.vtkRenderer()
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.SetWindowName("Axes")
    renderWindow.AddRenderer(renderer)

    # an interactor
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    # add the actors to the scene
    # ~ renderer.AddActor(sphereActor)
    renderer.SetBackground(colors.GetColor3d("SlateGray"))

    transform = vtk.vtkTransform()
    transform2 = vtk.vtkTransform()
    # ~ transform.Translate(1.0, 0.0, 0.0)

    axes = vtk.vtkAxesActor()
    axes2 = vtk.vtkAxesActor()
    #  The axes are positioned with a user transform
    axes.SetUserTransform(transform)
    axes2.SetUserTransform(transform2)

    # properties of the axes labels can be set as follows
    # this sets the x axis label to red
    # axes.GetXAxisCaptionActor2D().GetCaptionTextProperty().SetColor(colors.GetColor3d("Red"));

    # the actual text of the axis label can be changed:
    # axes->SetXAxisLabelText("test");

    renderer.AddActor(axes)
    renderer.AddActor(axes2)

    renderer.GetActiveCamera().Azimuth(30)
    renderer.GetActiveCamera().Elevation(30)

    renderer.ResetCamera()
    renderWindow.Render()

   # Sign up to receive TimerEvent
    cb = vtkTimerCallback(540, transform2, renderWindowInteractor)
    renderWindowInteractor.AddObserver('TimerEvent', cb.execute)
    cb.timerId = renderWindowInteractor.CreateRepeatingTimer(1000)

    # begin mouse interaction
    renderWindowInteractor.Start()


if __name__ == "__main__":
    main()
