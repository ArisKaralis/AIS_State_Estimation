function segmentAIS(data, mmsi, segmentIndex, sogThreshold, outputPath)
% SEGMENTAIS - Segment AIS data and save a specific segment to CSV

    segments = segmentMovingAIS(data, sogThreshold);
    
    if segmentIndex <= length(segments)
        segment = segments{segmentIndex};
        segment_out = formatAISTable(segment, mmsi);
        filename = fullfile(outputPath, sprintf('segment%d_ais_%d.csv', segmentIndex, mmsi));
        writetable(segment_out, filename);
        fprintf('Segment %d saved to %s\n', segmentIndex, filename);
    else
        warning('Only %d segments found. Requested segment %d not saved.', length(segments), segmentIndex);
    end
end

function segments = segmentMovingAIS(data, sog_threshold)
% SEGMENTMOVINGAIS - Split AIS data into moving segments (SOG > threshold)

    moving = data.SOG > sog_threshold;
    moving_diff = diff([0; moving(:); 0]);
    starts = find(moving_diff == 1);
    ends = find(moving_diff == -1) - 1;

    segments = {};
    for i = 1:length(starts)
        seg = data(starts(i):ends(i), :);
        if height(seg) >= 3
            segments{end+1} = seg;
        end
    end
end
