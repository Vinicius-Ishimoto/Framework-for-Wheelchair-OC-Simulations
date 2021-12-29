function [salpha, sbeta] = dof_conversion(alpha, beta, varargin)

% Conversion between static dof (horizontal plane as reference) to relative
% angles.
%
% Usage: [salpha, sbeta] = dof_conversion(alpha, beta, varargin)
%
% Parameters: 'Inverse' and 'Type'
% - dof_conversion(alpha, beta, 'Type', 'angle')
% for direct angle conversion
%
% - dof_conversion(alpha, beta, 'Type', 'angvel')
% for direct angular velocity conversion
%
% - dof_conversion(alpha, beta, 'Type', 'torque')
% for direct torque conversion
%

p = inputParser;

addParameter(p,'Inverse',false,@islogical);
addParameter(p,'Type','NaN');
parse(p,varargin{:});
inv = p.Results.Inverse;
type = p.Results.Type;

% Type: NaN (not defined)
if strcmp(type,'NaN')
    error('foo:bar','Please, define the type of conversion.\n- Possible types: angle, angvel, torque.');

% Type: angle
elseif strcmp(type,'angle')
    salpha = -(alpha-pi/2);
    sbeta = alpha-beta;
    
% Type: angvel
elseif strcmp(type,'angvel')
    salpha = -(alpha);
    sbeta = alpha-beta;
    
% Type: torque
elseif strcmp(type,'torque')
    salpha = -alpha;
    sbeta = -beta;
else
    error('I don''t know this type of conversion');
end