function[pose] = wrapAroundCorrection(pose)

while (pose(3)<0)
    pose(3) = pose(3)+2*pi;
end
while (pose(3)>2*pi)
    pose(3) = pose(3)-2*pi;
end

end