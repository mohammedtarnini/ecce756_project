clc; clear; close all;

OriginalVideo = VideoReader("C:\Users\moham\OneDrive - ku.ac.ae\University  Folders\Year 7\Robotic Perception\Project\Codes\Part A\Road_Videos\1\Clean\1.mp4");
EventsVideo = VideoReader("C:\Users\moham\OneDrive - ku.ac.ae\University  Folders\Year 7\Robotic Perception\Project\Codes\Part A\Road_Videos\1\Clean\dvs_video.mp4");

TrackedEvents = VideoWriter('tracked_events', 'MPEG-4');
TrackedOriginal = VideoWriter('tracking_original', 'MPEG-4');
TrackedEvents.FrameRate = min(EventsVideo.FrameRate, OriginalVideo.FrameRate);
TrackedOriginal.FrameRate = TrackedEvents.FrameRate;
open(TrackedEvents);
open(TrackedOriginal);

% Video player for side-by-side preview
videoPlayer = vision.VideoPlayer('Position', [100, 100, 1360, 520]); % Wider window for both videos

kalmanFilters = {};
ageCounter = [];
maxDistance = 20;
areaThreshold = 500;
maxTrackAge = 10;

while hasFrame(EventsVideo) && hasFrame(OriginalVideo)
    eventsFrame = readFrame(EventsVideo);
    originalFrame = readFrame(OriginalVideo);
    
    % Remove salt-and-pepper noise using a median filter
    denoisedFrame = medfilt3(eventsFrame, [3 3 3]); % Apply 3x3x3 median filter
    
    % Convert the denoised frame to grayscale
    grayFrame = rgb2gray(denoisedFrame);
    
    % Apply thresholding to separate the background
    thresholdValue = 120; % Adjust this value as needed
    binaryFrame = grayFrame > thresholdValue;
    
    % Invert the binary frame (make white black and black white)
    invertedBinaryFrame = ~binaryFrame;  % Invert the binary frame (flip black and white)
    
    % Convert logical to uint8 (0 = black, 255 = white)
    invertedBinaryFrame = uint8(invertedBinaryFrame) * 255;
    
    % Label connected regions in the binary frame
    labeledForeground = bwlabel(invertedBinaryFrame);
    stats = regionprops(labeledForeground, 'Centroid', 'Area'); % Get centroids and area of each cluster
    detections = cat(1, stats([stats.Area] > areaThreshold).Centroid); % Only consider clusters above area threshold

    assignedDetections = false(size(detections, 1), 1);

    % Initialize binary frame for visualization
    binaryFrameWithMarkers = repmat(invertedBinaryFrame, [1, 1, 3]);

    for i = 1:numel(kalmanFilters)
        predictedLocation = predict(kalmanFilters{i});

        if ~isempty(detections)
            distances = sqrt(sum((detections - predictedLocation).^2, 2));
            [minDist, idx] = min(distances);
            if minDist < maxDistance
                % Update Kalman filter with this detection
                correctedLocation = correct(kalmanFilters{i}, detections(idx, :));
                assignedDetections(idx) = true; % Mark detection as assigned
                ageCounter(i) = 0; % Reset age for active track

                % Add tracking markers
                binaryFrameWithMarkers = insertShape(binaryFrameWithMarkers, 'Circle', [predictedLocation, 13], 'Color', 'red', 'LineWidth', 2);
                binaryFrameWithMarkers = insertShape(binaryFrameWithMarkers, 'FilledCircle', [correctedLocation, 13], 'Color', 'green');
                originalFrame = insertShape(originalFrame, 'Circle', [predictedLocation, 13], 'Color', 'red', 'LineWidth', 2);
                originalFrame = insertShape(originalFrame, 'FilledCircle', [correctedLocation, 13], 'Color', 'green');
            else
                ageCounter(i) = ageCounter(i) + 1; % Increment age for inactive track
            end
        end
    end

    toRemove = ageCounter > maxTrackAge;
    kalmanFilters(toRemove) = [];
    ageCounter(toRemove) = [];

    % Add new detections if total tracks are below the limit
    for i = 1:size(detections, 1)
        if ~assignedDetections(i)
            if numel(kalmanFilters) < 20 % Limit to a maximum of 20 tracked objects
                initialLocation = detections(i, :);
                kalmanFilter = configureKalmanFilter('ConstantVelocity', initialLocation, ...
                                                     [1, 1], [0.1, 0.1], 1);
                kalmanFilters{end + 1} = kalmanFilter;
                ageCounter(end + 1) = 0;

                % Add tracking markers
                binaryFrameWithMarkers = insertShape(binaryFrameWithMarkers, 'FilledCircle', [initialLocation, 13], 'Color', 'green');
                originalFrame = insertShape(originalFrame, 'FilledCircle', [initialLocation, 13], 'Color', 'green');
            end
        end
    end

    % Write frames to output videos
    writeVideo(TrackedEvents, binaryFrameWithMarkers);
    writeVideo(TrackedOriginal, originalFrame);

    % Combine binary and overlay videos side by side for preview
    combinedPreview = [binaryFrameWithMarkers, originalFrame];
    step(videoPlayer, combinedPreview); % Display combined video
end

release(videoPlayer);
close(TrackedEvents);
close(TrackedOriginal);

disp('Processing complete. Videos saved as tracked_binary.avi and tracked_overlay.avi');
