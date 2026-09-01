%%% Command Window Progress bar
%%% Created By me(Dheeraj) at 28/03/2026 v0.0.1
function progressbar(stringORcVal, rangeVal)
persistent initialized range prevFullProgressBarLen;
if(isempty(initialized))
    initialized = false;
    range = [];
end
maxBars = 20;

if(nargin < 1)
    if(~initialized)
        error("Please initialize the progress bar first!");
    end

    error("Please input one argument as numerical value for displaying the current value!");
elseif(nargin < 2)
    if(~initialized)
        error("Please initialize the progress bar first!");
    end

    if(~isnumeric(stringORcVal) || sum(size(stringORcVal)) ~= 2)
        error("Please enter a valid current value i.e. scalar numeric value!");
    end

    if(stringORcVal < range(1) || stringORcVal > range(2))
        error("Plese enter a numeric value between specified range i.e. [%0.1f, %0.1f]", range(1), range(2));
    end

    fprintf(repmat('\b', 1, prevFullProgressBarLen));

    percent = (stringORcVal - range(1) + 1) * 100 / (range(2) - range(1) + 1);
    nBars = floor(0.2 * percent);

    progressStr = repmat('-', 1, maxBars);
    progressStr(1:nBars) = '|';
    progressStr = string(progressStr);

    fullProgressStr = sprintf("Progress: [%s] %0.2f%%", progressStr, percent);
    prevFullProgressBarLen = strlength(fullProgressStr);

    fprintf("%s", fullProgressStr);
    if(nBars == maxBars)
        fprintf("\n");
        initialized = false;
        range = [];
        prevFullProgressBarLen = NaN;
    end

    return;
elseif(nargin < 3)
    if(initialized)
        fprintf("\n");
        warning("Progress bar has already been initialized!"...
            + newline + "Initializing the progress bar again!");
    end

    if(isstring(stringORcVal) && isnumeric(rangeVal))
        if(stringORcVal ~= "Initialize")
            error("To initilalize please input the first argument as **Initialize** string.");
        end

        if(sum(size(rangeVal)) ~= 3 || rangeVal(1) >= rangeVal(2))
            error("Please enter a proper range which has total two entries and the first entry" ...
                + " is less than the second one.");
        end

        range = rangeVal;
        initialized = true;
        % fprintf("The progress bar has successfully initialized...\n");

        progressStr = repmat('-', 1, maxBars);
        progressStr = string(progressStr);

        fullProgressStr = sprintf("Progress: [%s] 0%%", progressStr);
        prevFullProgressBarLen = strlength(fullProgressStr);

        fprintf("%s", fullProgressStr);
        return;
    end
end

error("Please supply less than 3 inputs to the progressbar function!");
end