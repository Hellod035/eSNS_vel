figure

for i = 1:size(qSolutions,2)

    config = [qSolutions(1:7,i);0;0];
    show(robot,config,'Frames','off','PreservePlot',false,FastUpdate=true);
    hold on
    if i==1
        plot3(trajectory(1, :), trajectory(2, :), trajectory(3, :), 'b', 'LineWidth', 2);
        hold on
    end
    pause(0.01); 

end
