function [targs, targs_percam] = getDebugTargets(targs, targs_percam, people)
    id1 = targs(:,2) == people(1);
    id2 = targs(:,2) == people(2);
    id = id1 | id2;
    targs = targs(id,:);

    id1 = targs_percam{1}(:,2) == people(1);
    id2 = targs_percam{1}(:,2) == people(2);
    id = id1 | id2;
    targs_percam{1} = targs_percam{1}(id,:);

    id1 = targs_percam{2}(:,2) == people(1);
    id2 = targs_percam{2}(:,2) == people(2);
    id = id1 | id2;
    targs_percam{2} = targs_percam{2}(id,:);

end
