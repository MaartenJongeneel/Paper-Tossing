function textwaitbar(i, n, msg, astime)
% A command line version of waitbar.
% Usage:
%   textwaitbar(i, n, msg)
% Input:
%   i   :   i-th iteration.
%   n   :   total iterations.
%   msg :   text message to print.
%
% Date      : 05/23/2019
% Author    : Xiaoxuan He   <hexxx937@umn.edu>
% Institute : University of Minnesota
% >> adapted for time display by Luuk Poort TU/e 28/02/2020

% Previous percentage number.
persistent i_prev_prct;

% Print as percentage unless specified.
if ~exist('astime','var'), astime = false; end

% Current percentage/time number.
if astime
    i_prct = round(i,2);
else
    i_prct = floor(i ./ n * 100);
end

% Print message when counting starts.
if isempty(i_prev_prct) %|| i_prct ~= i_prev_prct
    i_prev_prct = 0;
    S_prev = getPrctStr(i_prev_prct,astime,i);
    
    fprintf('%s: %s',msg, S_prev);
end

% Print updated percentage.
if i_prct ~= i_prev_prct
    S_prev = getPrctStr(i_prev_prct,astime,i);
    fprintf(getBackspaceStr(numel(S_prev)));
    
    S = getPrctStr(i_prct,astime,i);
    fprintf('%s', S);
    
    i_prev_prct = i_prct;
end

% Clear percentage variable.
if round(i,3)==round(n,3)
    fprintf(' Done.\n');
    clear i_prev_prct;
end

end

function S = getPrctStr(prct,astime,t)
if astime
    S = sprintf(' %.2fs  %s',t,getDotStr(prct));
else
    S = sprintf('  %d%%  %s',prct,getDotStr(prct));
end

if prct < 10
    S = ['  ',S];
elseif prct < 100
    S = [' ',S];
end
end

function S = getDotStr(prct)
S = repmat(' ',1,10);
S(1:floor(prct/10)) = '.';
S = ['[',S,']'];
end

function S = getBackspaceStr(N)
S = repmat('\b',1,N);
end