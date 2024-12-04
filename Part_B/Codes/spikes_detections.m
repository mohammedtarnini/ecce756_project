clc; clear; close all;

OriginalVideo = VideoReader("C:\Users\moham\OneDrive - ku.ac.ae\University  Folders\Year 7\Robotic Perception\Project\Codes\Part B\Pool_Videos\4\Clean\4.mp4");
SpikeVideo = VideoReader("C:\Users\moham\OneDrive - ku.ac.ae\University  Folders\Year 7\Robotic Perception\Project\Codes\Part B\Pool_Videos\4\Clean\Spikes_Output.avi");
TrackedOriginal = VideoWriter('track_original', 'MPEG-4');
TrackedSpike = VideoWriter('track_spike', 'MPEG-4');

frameRate = min([OriginalVideo.FrameRate, SpikeVideo.FrameRate]);
TrackedOriginal.FrameRate = frameRate;
TrackedSpike.FrameRate = frameRate;

open(TrackedOriginal);
open(TrackedSpike);

videoPlayer = vision.VideoPlayer('Position', [100, 100, 960, 520]);
areaThreshold = 300;
mergeThreshold = 200;

while hasFrame(SpikeVideo) && hasFrame(OriginalVideo)
    spikeFrame = readFrame(SpikeVideo);
    originalFrame = readFrame(OriginalVideo);
    graySpike = rgb2gray(spikeFrame);
    binarySpike = graySpike > 40;

    connComp = bwconncomp(binarySpike);
    stats = regionprops(connComp, 'Centroid', 'Area', 'PixelIdxList', 'BoundingBox');
    validStats = stats([stats.Area] > areaThreshold);

    rectangles = [];

    % Compute bounding boxes for each detection
    for i = 1:length(validStats)
        boundingBox = validStats(i).BoundingBox;
        rectangles = [rectangles; boundingBox, validStats(i).Area]; % Append [x, y, width, height, area]
    end

    % Initialize list for merged rectangles
    mergedRectangles = [];
    toRemove = false(size(rectangles, 1), 1);

    % Check for proximity and intersection to merge rectangles
    for i = 1:size(rectangles, 1)
        if toRemove(i)
            continue;
        end
        rectI = rectangles(i, 1:4); % [x, y, width, height]
        
        % Check for proximity and intersection with other rectangles
        for j = i+1:size(rectangles, 1)
            if toRemove(j)
                continue;
            end
            rectJ = rectangles(j, 1:4); % [x, y, width, height]
            overlapX = max(0, min(rectI(1) + rectI(3), rectJ(1) + rectJ(3)) - max(rectI(1), rectJ(1)));
            overlapY = max(0, min(rectI(2) + rectI(4), rectJ(2) + rectJ(4)) - max(rectI(2), rectJ(2)));
            overlapArea = overlapX * overlapY;
            
            if overlapArea > 0 || norm([rectI(1) + rectI(3)/2, rectI(2) + rectI(4)/2] - [rectJ(1) + rectJ(3)/2, rectJ(2) + rectJ(4)/2]) < mergeThreshold
                newRect = [
                    min(rectI(1), rectJ(1)), ... 
                    min(rectI(2), rectJ(2)), ...
                    max(rectI(1) + rectI(3), rectJ(1) + rectJ(3)) - min(rectI(1), rectJ(1)), ...
                    max(rectI(2) + rectI(4), rectJ(2) + rectJ(4)) - min(rectI(2), rectJ(2))
                ];
                
                mergedRectangles = [mergedRectangles; newRect, rectangles(i, 5) + rectangles(j, 5)];
                toRemove([i, j]) = true; % Mark both rectangles for removal
                break; % No need to check further for this pair
            end
        end
        
        if ~toRemove(i)
            mergedRectangles = [mergedRectangles; rectI, rectangles(i, 5)];
        end
    end

    % Draw detections and bounding boxes
    spikeFrameWithDetections = repmat(uint8(binarySpike) * 255, [1, 1, 3]);
    originalFrameWithBoxes = originalFrame;

    for i = 1:size(mergedRectangles, 1)
        rect = mergedRectangles(i, 1:4); % [x, y, width, height]
        
        % Draw bounding box on both frames
        spikeFrameWithDetections = insertShape(spikeFrameWithDetections, 'Rectangle', ...
            rect, 'Color', 'red', 'LineWidth', 2);
        originalFrameWithBoxes = insertShape(originalFrameWithBoxes, 'Rectangle', ...
            rect, 'Color', 'blue', 'LineWidth', 2);
    end

    writeVideo(TrackedSpike, spikeFrameWithDetections);
    writeVideo(TrackedOriginal, originalFrameWithBoxes);
    combinedPreview = [spikeFrameWithDetections, originalFrameWithBoxes];
    step(videoPlayer, combinedPreview);
end

release(videoPlayer);
close(TrackedSpike);
close(TrackedOriginal);