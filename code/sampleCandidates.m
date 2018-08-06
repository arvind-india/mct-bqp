function [cands, cands_percam] = sampleCandidates(N, k, num_cams, targs, homographies, invhomographies, sampling_plane, dx, dy)
    cands = cell(N,1);
    cands_percam = cell(num_cams,1); % Candidates from both cameras for each target
    for t = 1:size(targs,1)
        % t_pos = targs(t,8:9);
        t_rect = targs(t,4:7);
        cands{t} = zeros(k,6);

        i = 1;
        for gridx = -floor(sqrt(k)/2):floor(sqrt(k)/2)
          for gridy = -floor(sqrt(k)/2):floor(sqrt(k)/2)
            if strcmp(sampling_plane,'camera')
                sampling_dx = t_rect(3)/dx;
                sampling_dy = t_rect(4)/dy;
                cx = t_rect(1) + gridx * sampling_dx;
                cy = t_rect(2) + gridy * sampling_dy;
                cands{t}(i,:) = [cx cy t_rect(3:4) transpose(H(homographies{targs(t,1)}, [cx+t_rect(3)/2; cy+t_rect(4)]))];
            %elseif strcmp(sampling_plane,'ground')
            %    sampling_dx = g_dx; % TODO Bernardino proposed a way to actually get bb width from gnd plane
            %    sampling_dy = g_dy; % TODO but we would need some data we do not have, so we would just assume bbs :(
            %    g_cx = t_pos(1) + gridx * sampling_dx;
            %    g_cy = t_pos(2) + gridy * sampling_dy;
            %    c_pos = transpose(H(invhomographies{targs(t,1)}, [g_cx; g_cy]));
            %    cands{t}(i,:) = [c_pos(1) c_pos(2) t_rect(3:4) g_cx g_cy];
            end
            i = i + 1;
          end
        end
        cands_percam{targs(t,1)}{end+1} = cands{t};
    end
end
