clc; clear; close all;

OriginalVideo = VideoReader("C:\Users\moham\OneDrive - ku.ac.ae\University  Folders\Year 7\Robotic Perception\Project\Codes\Part B\Pool_Videos\4\Clean\4.mp4");
EventsVideo = VideoReader("C:\Users\moham\OneDrive - ku.ac.ae\University  Folders\Year 7\Robotic Perception\Project\Codes\Part B\Pool_Videos\4\Clean\dvs_video.mp4");

TrackedEvents = VideoWriter('track_v2e', 'MPEG-4');
TrackedOriginal = VideoWriter('track_original', 'MPEG-4');
TrackedEvents.FrameRate = min(EventsVideo.FrameRate, OriginalVideo.FrameRate);
TrackedOriginal.FrameRate = TrackedEvents.FrameRate;
open(TrackedEvents);
open(TrackedOriginal);

videoPlayer = vision.VideoPlayer('Position', [100, 100, 1360, 520]); % Wider window for both videos

areaThreshold = 300;
mergeDistanceThreshold = 300;
maxDetections = 6;

while hasFrame(EventsVideo) && hasFrame(OriginalVideo)
    eventsFrame = readFrame(EventsVideo);
    originalFrame = readFrame(OriginalVideo);
    denoisedFrame = medfilt2(eventsFrame, [3 3]);
    grayFrame = rgb2gray(denoisedFrame);
    thresholdValue = 120;
    binaryFrame = grayFrame > thresholdValue;
    filteredBinaryFrame = medfilt2(binaryFrame, [7 7]);
    invertedBinaryFrame = ~filteredBinaryFrame;
    invertedBinaryFrame = uint8(invertedBinaryFrame) * 255;
    
    labeledForeground = bwlabel(invertedBinaryFrame);
    stats = regionprops(labeledForeground, 'Centroid', 'Area', 'BoundingBox'); % Get bounding box
    validStats = stats([stats.Area] > areaThreshold); % Only consider clusters above area threshold
    boundingBoxes = cat(1, validStats.BoundingBox);
    mergedBoundingBoxes = boundingBoxes;
    numBoxes = size(boundingBoxes, 1);
    for i = 1:numBoxes
        for j = i + 1:numBoxes
            if i > size(mergedBoundingBoxes, 1) || j > size(mergedBoundingBoxes, 1)
                continue;
            end
            box1 = mergedBoundingBoxes(i, :);
            box2 = mergedBoundingBoxes(j, :);
            
            % Calculate the center of the two rectangles
            center1 = [box1(1) + box1(3) / 2, box1(2) + box1(4) / 2];
            center2 = [box2(1) + box2(3) / 2, box2(2) + box2(4) / 2];
            
            % Check the distance between the centers
            distance = norm(center1 - center2);
            if distance < mergeDistanceThreshold
                xMin = min(box1(1), box2(1));
                yMin = min(box1(2), box2(2));
                xMax = max(box1(1) + box1(3), box2(1) + box2(3));
                yMax = max(box1(2) + box1(4), box2(2) + box2(4));
                mergedBoundingBoxes(i, :) = [xMin, yMin, xMax - xMin, yMax - yMin];
                mergedBoundingBoxes(j, :) = [];
                numBoxes = numBoxes - 1;
                break;
            end
        end
    end

    if size(mergedBoundingBoxes, 1) > maxDetections
        areas = mergedBoundingBoxes(:, 3) .* mergedBoundingBoxes(:, 4);
        [~, sortedIndices] = sort(areas, 'descend');
        mergedBoundingBoxes = mergedBoundingBoxes(sortedIndices(1:maxDetections), :);
    end

    % Initialize binary frame for visualization
    binaryFrameWithMarkers = repmat(invertedBinaryFrame, [1, 1, 3]);

    for i = 1:size(mergedBoundingBoxes, 1)
        box = mergedBoundingBoxes(i, :);
        binaryFrameWithMarkers = insertShape(binaryFrameWithMarkers, 'Rectangle', ...
            box, 'Color', 'yellow', 'LineWidth', 2);
        originalFrame = insertShape(originalFrame, 'Rectangle', ...
            box, 'Color', 'yellow', 'LineWidth', 2);
    end

    writeVideo(TrackedEvents, binaryFrameWithMarkers);
    writeVideo(TrackedOriginal, originalFrame);
    combinedPreview = [binaryFrameWithMarkers, originalFrame];
    step(videoPlayer, combinedPreview); % Display combined video
end

release(videoPlayer);
close(TrackedEvents);
close(TrackedOriginal);