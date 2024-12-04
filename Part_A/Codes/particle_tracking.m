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

particleFilters = {};  % Cell array to hold particle filter data
ageCounter = [];
maxDistance = 2;
areaThreshold = 50;
maxTrackAge = 10;
numParticles = 100; % Number of particles per object
spreadRadius = 5; % Spread radius for particles
noiseLevel = 10; % Increased noise level for faster particle movement

while hasFrame(EventsVideo) && hasFrame(OriginalVideo)
    eventsFrame = readFrame(EventsVideo);
    originalFrame = readFrame(OriginalVideo);
    
    % Remove salt-and-pepper noise using a median filter (for grayscale image)
    denoisedFrame = medfilt3(eventsFrame, [5 5 5]); % Apply 3x3x3 median filter to the original frame
    
    % Convert the denoised frame to grayscale
    grayFrame = rgb2gray(denoisedFrame);
    
    % Apply thresholding to separate the background
    thresholdValue = 100; % Adjust this value as needed
    binaryFrame = grayFrame > thresholdValue;
    
    % Apply a 2D median filter to remove salt-and-pepper noise from the binary frame
    binaryFrame = medfilt2(binaryFrame, [5 5]); % Apply 3x3 median filter for binary noise removal
    
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

    % Update particle filters for each object
    for i = 1:length(particleFilters)
        if ~isempty(detections)
            % Get the actual detection for the current object
            detectionIndex = min(i, size(detections, 1));  % Ensure i doesn't exceed detections size
            detection = detections(detectionIndex, :); 
            
            % Compute the distance between the particles and the detected centroid
            distances = sqrt(sum((particleFilters{i}.particles - detection).^2, 2)); % Euclidean distance
            likelihood = exp(-distances.^2 / (2 * spreadRadius^2)); % Likelihood based on distance
            
            % Ensure non-negative and non-zero weights
            likelihood = max(likelihood, eps);  % Ensure no negative weights
            particleFilters{i}.weights = likelihood / sum(likelihood); % Normalize the weights to sum to 1
            
            % Resample particles based on the weights
            indices = randsample(1:numParticles, numParticles, true, particleFilters{i}.weights);
            particleFilters{i}.particles = particleFilters{i}.particles(indices, :); % Resample particles
            
            % Propagate particles by adding noise (simulating movement) and make them move faster
            particleFilters{i}.particles = particleFilters{i}.particles + randn(numParticles, 2) * noiseLevel; % Increased speed
            
            % Compute the estimated position as the weighted average of particles
            estimatedPosition = sum(particleFilters{i}.particles .* particleFilters{i}.weights, 1);
            
            % Display particles (red circles) for binary frame
            binaryFrameWithMarkers = insertShape(binaryFrameWithMarkers, 'Circle', [particleFilters{i}.particles, repmat(2, numParticles, 1)], 'Color', 'red', 'LineWidth', 1);
            
            % Display the estimated position (red circle) for the original frame
            originalFrame = insertShape(originalFrame, 'Circle', [estimatedPosition, 2], 'Color', 'red', 'LineWidth', 1);
            
            % Display particles on the original frame (same as for binary frame)
            originalFrame = insertShape(originalFrame, 'Circle', [particleFilters{i}.particles, repmat(2, numParticles, 1)], 'Color', 'red', 'LineWidth', 1);

            % Add thick green "+" symbol at the centroid of each neighborhood
            plusSize = 15; % Length of the plus symbol lines
            lineThickness = 3; % Thickness of the lines for the "+" symbol
            
            % Create vertical and horizontal lines for the "+" symbol
            verticalLine = [detection(1), detection(2) - plusSize / 2, detection(1), detection(2) + plusSize / 2];
            horizontalLine = [detection(1) - plusSize / 2, detection(2), detection(1) + plusSize / 2, detection(2)];

            % Insert the "+" symbol into the original frame and binary frame
            originalFrame = insertShape(originalFrame, 'Line', verticalLine, 'Color', 'green', 'LineWidth', lineThickness);
            originalFrame = insertShape(originalFrame, 'Line', horizontalLine, 'Color', 'green', 'LineWidth', lineThickness);
            
            binaryFrameWithMarkers = insertShape(binaryFrameWithMarkers, 'Line', verticalLine, 'Color', 'green', 'LineWidth', lineThickness);
            binaryFrameWithMarkers = insertShape(binaryFrameWithMarkers, 'Line', horizontalLine, 'Color', 'green', 'LineWidth', lineThickness);
        end
    end

    toRemove = ageCounter > maxTrackAge;
    particleFilters(toRemove) = [];
    ageCounter(toRemove) = [];

    % Add new detections if total tracks are below the limit
    for i = 1:size(detections, 1)
        if ~assignedDetections(i)
            if numel(particleFilters) < 20 % Limit to a maximum of 20 tracked objects
                initialLocation = detections(i, :);
                % Initialize a particle filter for each new object
                particleFilters{end + 1}.centroid = initialLocation; % Initial centroid
                particleFilters{end}.particles = repmat(initialLocation, numParticles, 1) + randn(numParticles, 2) * spreadRadius; % Spread particles
                particleFilters{end}.weights = ones(numParticles, 1) / numParticles; % Initialize uniform weights
                ageCounter(end + 1) = 0;
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

disp('Processing complete. Videos saved as tracked_events and tracking_original.');
