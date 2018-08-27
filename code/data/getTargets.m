function  [targs, targs_percam] = getTargets(f, detection_frames, gnd_detections, num_cams, start_frames)
    if ismember(f,detection_frames)
        targs = cell(num_cams,1); % Targets from all cameras
        for id = 1:num_cams
            targs{id} = gnd_detections{id}{start_frames(id) + f};
        end
        targs = cell2mat(targs); % For some reason this work on one computer and on the other one does not
        if isempty(targs) % NOTE If the targets are empty on the first frame, can't track
            error('Cannot possibly track if there are no detections on the first given frame');
        end
    end
    targs_percam = (accumarray(targs(:,1),(1:size(targs,1)).',[],@(x){targs(x,:)},{}));
    targs_percam = targs_percam(~cellfun('isempty',targs_percam));
end
