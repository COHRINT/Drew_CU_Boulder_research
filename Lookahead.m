function [utility,pix,piy] = Lookahead(A,x,y,dist,w,wcurrent)                       %Define function name, outputs and inputs: A is the grid, x & y are the current position, dist is the number of tiles it can move, w is the number of lookahead steps used, and wcurrent is always set to 1 by the user                                                                   %Initialize utility (probability)
utility = 0; 
utilityMax = 0;
[m,n] = size(A);                                                               %Extract dimensions of grid
for i= -dist:dist,                                                              %Start searching throughout grid, limited to distance that agent can move in one time step
    for j=-dist:dist,
        if ((x+j >= 1) && (y+i >= 1)&&(x+j)<=n && (y+i)<=m)                     %Only look in valid grid cells
            if ~isnan(A(y+i,x+j))
                currentBelief = A(y+i,x+j);                                         %Extract belief probability of current cell
                if(wcurrent < w)                                                   %If we have another step in Lookahead, run the function again
                    [utility2,~,~] = Lookahead(A,x+j,y+i,dist,w,wcurrent+1);           %Extract data from subroutine (nra: replaced unused outputs with ~)
                    currentBelief = currentBelief + utility2;                         %Then update total utility of current Lookahead
                    if currentBelief > utility                                      %If the current lookahead creates a high utility, update the optimal path parameters
                        utility = currentBelief;
                        pix = x+j;
                        piy=y+i;
                    end
                else                                                               %If it is the last step in Lookahead
                    currentBelief = utility + currentBelief;
                    if currentBelief > utilityMax                                        %And the current belief is more optimal, update the the optimal path parameters
                        utilityMax = currentBelief;
                        pix = x+j;
                        piy=y+i;
                    end
                end
            end
        end
    end
end