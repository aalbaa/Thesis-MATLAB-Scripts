function S = sylvester(P,Q ,MpMq)
% SYLVESTER - Sylvester matrix of two polynomials
%   S = SYLVESTER(P,Q) returns the Sylvester matrix S that is associated
%   with the two polynomial representations P and Q, of degree Dp and Dq,
%   respectively. S is a (Dp+Dq)-by-(Dp+Dq) square matrix. Note that the
%   degree of the polynomial is one less the the number of elements of its
%   representation: degree(P) = numel(P)-1.
%
%   SYLVESTER(P,Q, [Mp Mq]) returns a generalization by specifiying the size.
%   The Sylvester matrix S will be a, in general non-square, (Mp+Mq)-by-K
%   matrix, with K being the maximum of (Mp+Dp) and (Mq+Dq). 
%
%   Examples:
%     P = [1 2 3 4] ; Q =  [6 7] ; % Dp = 3, Dq = 1 ;
%     M = sylvester(P,Q)
%      %  1     2     3     4
%      %  6     7     0     0
     %  0     6     7     0
%      %  0     0     6     7
%
%     M = sylvester([1 0 3],[2 4],[3 1])
%      % -> a non-square sylvester matrix
%      %  1     0     3     0     0
%      %  0     1     0     3     0
%      %  0     0     1     0     3
%      %  2     4     0     0     0
%
%   For more information, see http://en.wikipedia.org/wiki/Sylvester_matrix
%
%   See also GALLERY, TOEPLITZ
%            CIRCULANT (File Exchange)

% for Matlab R13 and up
% version 3.0 (jun 2013)
% (c) Jos van der Geest
% email: jos@jasen.nl

% History
% 1.0 (may 2009) - created, inpsired by a post on CSSM
% 2.0 (dec 2009) - added size option, suggested by Julian Stoev
% (julian.stoev@gmail.com)
% 3.0 (jun 2013) - fixed the error mentioned by Paul, sylvester(P,Q) now
%                  always return a square matrix

error(nargchk(2,3,nargin)) ;

NP = numel(P) ;
NQ = numel(Q) ;

if nargin==3 
    if ~isnumeric(MpMq) || numel(MpMq) ~= 2 || any(MpMq ~= fix(MpMq)) || any(MpMq < 1)
       error('Size argument should be a 2-element vector with positive integers.') ; 
    end
    % Note the switch of N between P and Q
    NP = MpMq(2) + 1 ;
    NQ = MpMq(1) + 1 ;
end
    
if ~isnumeric(P) || ~isnumeric(Q) || NP==0 || NQ==0,
    error('Input arguments should be numeric vectors.') ;
end

% turn off TOEPLITZ warning
WS = warning('off','MATLAB:toeplitz:DiagonalConflict') ;

% create sub-matrices (fixed in jun 2013)
SP = toeplitz([P(:) ; zeros(NQ-2,1)],zeros(NQ-1,1)) ;
SQ = toeplitz([Q(:) ; zeros(NP-2,1)],zeros(NP-1,1)) ;

if nargin==3,
    % add appropriate rows of zeros to make the two submatrices the same
    % size
    DN = size(SP,1) - size(SQ,1) ;
    SP = [SP ; zeros(-DN,NQ-1)] ;
    SQ = [SQ ; zeros(DN,NP-1)] ;
end

% concatenate into Sylvester matrix
S = [SP SQ].' ;

warning(WS.state,WS.identifier)