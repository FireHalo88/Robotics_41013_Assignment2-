function [coefVec] = ImageProc()
%Mostly copied from
%https://au.mathworks.com/matlabcentral/answers/427291-how-would-i-return-a-matrix-from-a-function\

    clc;    % Clear the command window.
    close all;  % Close all figures (except those of imtool.)
    imtool close all;  % Close all imtool figures.
    clear;  % Erase all existing variables.
    workspace;  % Make sure the workspace panel is showing.
    format longg;
    format compact;
    fontSize = 20;

    % Read in a Vrinda's gray scale demo image.
    folder = ['C:\Users\VIncent\Documents\GitHub\Robotics_41013_Assignment2-\Image Processing test folder\' ...
        ;]
    baseFileName = 'image.jpeg';
    %baseFileName = 'test_bw_image.png';
    % Get the full filename, with path prepended.
    fullFileName = fullfile(folder, baseFileName);
    % Check if file exists.
    if ~exist(fullFileName, 'file')
        % File doesn't exist -- didn't find it there.  Check the search path for it.
        fullFileNameOnSearchPath = baseFileName; % No path this time.
        if ~exist(fullFileNameOnSearchPath, 'file')
            % Still didn't find it.  Alert user.
            errorMessage = sprintf('Error: %s does not exist in the search path folders.', fullFileName);
            uiwait(warndlg(errorMessage));
            return;
        end
    end
    grayImage = imread(fullFileName);
    % Get the dimensions of the image.  
    % numberOfColorBands should be = 1.
    [rows, columns, numberOfColorBands] = size(grayImage);
    if numberOfColorBands > 1
        % It's not really gray scale like we expected - it's color.
        % Convert it to gray scale by taking only the green channel.
        grayImage = grayImage(:, :, 2); % Take green channel.
    end
    % Display the original gray scale image.
    subplot(2, 2, 1);
    imshow(grayImage, []);
    title('Original Grayscale Image', 'FontSize', fontSize);
    % Enlarge figure to full screen.
    set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
    % Give a name to the title bar.
    set(gcf, 'Name', 'Demo by ImageAnalyst', 'NumberTitle', 'Off') 
    % Binarize to get rid of horrible jpeg noise
    binaryImage = grayImage < 100;
    % Display the original gray scale image.
    subplot(2, 2, 2);
    imshow(binaryImage, []);
    title('Binary Image', 'FontSize', fontSize);
    axis on;
    % Get the rows (y) and columns (x).
    [rows, columns] = find(binaryImage);
    % Fit a cubic
    coefficients = polyfit(columns, rows, 4); % Gets coefficients of the formula.
    % Fit a curve to 500 points in the range that x has.
    fittedX = linspace(min(columns), max(columns), 500);
    % Now get the y values.
    fittedY = polyval(coefficients, fittedX);
    % Plot the fitting:
    subplot(2,2,3:4);
    plot(fittedX, fittedY, 'b-', 'linewidth', 4);
    grid on;
    xlabel('X', 'FontSize', fontSize);
    ylabel('Y', 'FontSize', fontSize);
    % Overlay the original points in red.
    hold on;
    plot(columns, rows, 'r+', 'LineWidth', 2, 'MarkerSize', 10);
    coefVec = coefficients;
end

