function unstructuredLidarDataSet = structured2unstructuredPointCloud(lidarDataSet, removingInvalidPoints)
%This function is to convert recorded structured(MxNx3) lidar point
%cloud data set to unstructured (Mx3) point cloud format 

    arguments
        lidarDataSet
        removingInvalidPoints(1,1) {mustBeFinite} = false;
    end

    lidarDataSet_unstructured{size(lidarDataSet,1),1}=[]; % allocation

    for k=1:size(lidarDataSet,1) 
        for m=1:size(lidarDataSet{1,1}.Location,1)
            clear lidarData_temp; lidarData_temp = [];
            for p=1:size(lidarDataSet{1,1}.Location,3)
                lidarData_temp =...
                    cat(2, lidarData_temp,transpose(lidarDataSet{k,1}.Location(m,:,p)));
            end
            lidarDataSet_unstructured{k,1} = ...
                cat(1, lidarDataSet_unstructured{k,1}, lidarData_temp);
        end
    end

    unstructuredLidarDataSet{size(lidarDataSet_unstructured,1),1}=[]; % allocation
    for list = 1:size(lidarDataSet_unstructured,1)
        if removingInvalidPoints == true
            unstructuredLidarDataSet{list,1} =...
                removeInvalidPoints(pointCloud(lidarDataSet_unstructured{list,1}));
        else
            unstructuredLidarDataSet{list,1} = pointCloud(lidarDataSet_unstructured{list,1});
        end
    end
end