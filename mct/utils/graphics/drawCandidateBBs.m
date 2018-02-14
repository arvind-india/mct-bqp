function drawCandidateBBs(dets, bb_color, dataset, g)
    % Draw a bounding box given a detection and a colour
    % Inputs
    %  dets: numerical values of the detection
    %  colour: colour in rgb triplets
    if strcmp(dataset, 'campus_2')
      bb_offset = 12;
      bb_linewidth = 1;
      for k = 1:size(dets,1)
          startx = dets(k,1);
          starty = dets(k,2);
          bb_width = dets(k,3);
          bb_height = dets(k,4);
          xstep = bb_width/10;
          ystep = bb_height/10;
          for gridx=-2:2
              for gridy=-2:2
                  cx = startx + gridx*xstep;
                  cy = starty + gridy*ystep;
                  %bbpos{sqrt(k)*gridx+gridy} = [bbx bby bbwidth bbheight];
                  hold on
                  if cy+bb_height <= 800
                    rectangle('Position',[cx cy bb_width bb_height],'EdgeColor', bb_color, 'LineWidth', bb_linewidth);
                  else
                    %rectangle('Position',[cx cy bb_width bb_height],'EdgeColor', 'Red', 'LineWidth', bb_linewidth);
                  end
              end
          end
      end
    elseif strcmp(dataset, 'hda')
      bb_offset = 12;
      bb_linewidth = 1;
      for k = 1:size(dets,1)
          startx = dets(k,1);
          starty = dets(k,2);
          bb_width = dets(k,3);
          bb_height = dets(k,4);
          xstep = bb_width/10;
          ystep = bb_height/10;
          for gridx=-2:2
              for gridy=-2:2
                  cx = startx + gridx*xstep;
                  cy = starty + gridy*ystep;
                  %bbpos{sqrt(k)*gridx+gridy} = [bbx bby bbwidth bbheight];
                  hold on
                  if cy+bb_height <= 800
                    rectangle('Position',[cx cy bb_width bb_height],'EdgeColor', bb_color, 'LineWidth', bb_linewidth);
                  else
                    %rectangle('Position',[cx cy bb_width bb_height],'EdgeColor', 'Red', 'LineWidth', bb_linewidth);
                  end
              end
          end
      end

    end
