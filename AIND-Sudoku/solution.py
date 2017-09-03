
# coding: utf-8

# In[1]:

def cross(A, B):
    "Cross product of elements in A and elements in B."
    return [s+t for s in A for t in B]


# In[2]:

def display(values):
    """
    Display the values as a 2-D grid.
    Args:
        values(dict): The sudoku in dictionary form
    """
    width = 1+max(len(values[s]) for s in boxes)
    line = '+'.join(['-'*(width*3)]*3)
    for r in rows:
        print(''.join(values[r+c].center(width)+('|' if c in '36' else '')
                      for c in cols))
        if r in 'CF': print(line)
    return


# In[3]:

assignments = []
#Labels for sudoku board
rows = 'ABCDEFGHI'
cols = '123456789'

#lists of individual boxes
boxes = cross(rows, cols)
#sudoku board units
row_units = [cross(r,cols) for r in rows]
column_units = [cross(rows, c) for c in cols]
square_units = [cross(rs,cs) for rs in ('ABC', 'DEF', 'GHI') for cs in ('123', '456', '789')]

#check for diagonal solution
diagonal_units = [[rows[idx] + cols[idx] for idx in range(len(rows))],
    [rows[idx] + cols[-idx-1] for idx in range(len(rows))]]
#list of all units
unitlist = row_units + column_units + square_units + diagonal_units
units = dict((s, [u for u in unitlist if s in u]) for s in boxes)
#peer lookup dictionary, peer[box] is all boxes in same row, col, 3x3, or diagnoal
peers = dict((s, set(sum(units[s],[]))-set([s])) for s in boxes)

def assign_value(values, box, value):
    """
    Please use this function to update your values dictionary!
    Assigns a value to a given box. If it updates the board record it.
    """

    # Don't waste memory appending actions that don't actually change any values
    if values[box] == value:
        return values

    values[box] = value
    if len(value) == 1:
        assignments.append(values.copy())
    return values


# In[4]:

def naked_twins(values):
    """Eliminate values using the naked twins strategy.
    Args:
        values(dict): a dictionary of the form {'box_name': '123456789', ...}

    Returns:
        the values dictionary with the naked twins eliminated from peers.
    """
    
    def are_twins(a,b):
        """Check if two box possible values are twins, ignore order.
        Args:
            a(string): possible values of current box
            b(bool): possible values of peer box

        Returns:
            (Bool) True if the boxes contain exactly the same possible values, else False
        """
        return set(a) == set(b)
    
    # Find all instances of naked twins
    # Eliminate the naked twins as possibilities for their peers
    
    from collections import Counter
    
    for unit in unitlist:
        #naked twins based on counts in unit
        values_counts = Counter([values[box] for box in unit if len(values[box])==2])
        twins = [k for k,v in values_counts.items() if v == 2]
        #itterate through twins and boxes, remove twin characters from non-twins
        for twin in twins:
            for box in unit:
                value = values[box]
                if(not are_twins(value, twin)):
                    #remove twin characters
                    value = value.translate(twin.maketrans('', '', twin))
                    values = assign_value(values, box, value)
    return values


# In[5]:

def grid_values(grid):
    """
    Convert grid into a dict of {square: char} with '123456789' for empties.
    Args:
        grid(string) - A grid in string form.
    Returns:
        A grid in dictionary form
            Keys: The boxes, e.g., 'A1'
            Values: The value in each box, e.g., '8'. If the box has no value, then the value will be '123456789'.
    """
    empty_box_string = '123456789'   
    return {boxes[idx]:(value if value != '.' else empty_box_string) for idx,value in enumerate(grid)}


# In[6]:

def eliminate(values):
    """Eliminate values from peers of each box with a single value.

    Go through all the boxes, and whenever there is a box with a single value,
    eliminate this value from the set of values of all its peers.

    Args:
        values: Sudoku in dictionary form.
    Returns:
        Resulting Sudoku in dictionary form after eliminating values.
    """
    solved_values = [box for box in values.keys() if len(values[box]) == 1]
    for box in solved_values:
        digit = values[box]
        for peer in peers[box]:
            values = assign_value(values, peer, values[peer].replace(digit,''))
    return values


# In[7]:

def only_choice(values):
    """Finalize all values that are the only choice for a unit.

    Go through all the units, and whenever there is a unit with a value
    that only fits in one box, assign the value to this box.

    Args: 
        values: Sudoku in dictionary form.
    Returns: 
        Resulting Sudoku in dictionary form after filling in only choices.
    """
    for unit in unitlist:
        for digit in '123456789':
            dplaces = [box for box in unit if digit in values[box]]
            if len(dplaces) == 1:
                values = assign_value(values, dplaces[0], digit)
    return values


# In[9]:

def reduce_puzzle(values):
    """
    Iterate eliminate() and only_choice(). If at some point, there is a box with no available values, return False.
    If the sudoku is solved, return the sudoku.
    If after an iteration of both functions, the sudoku remains the same, return the sudoku.
    Args: 
        A sudoku in dictionary form.
    Return: 
        The resulting sudoku in dictionary form.
    """
    stalled = False
    while not stalled:
        # Check how many boxes have a determined value
        solved_values_before = len([box for box in values.keys() if len(values[box]) == 1])
        
        #run reduction strategies
        values = eliminate(values)
        values = only_choice(values)
        values = naked_twins(values)
        
        # Check how many boxes have a determined value, to compare
        solved_values_after = len([box for box in values.keys() if len(values[box]) == 1])
        # If no new values were added, stop the loop.
        stalled = solved_values_before == solved_values_after
        # Sanity check, return False if there is a box with zero available values:
        if len([box for box in values.keys() if len(values[box]) == 0]):
            return False
    return values


# In[10]:

def search(values):
    """Depth first search of possible sudoku solutions. Choose a square with fewest possible solutions,
        try a value and reduce recurisvely until solution is found or returns False for no solution
    Args:
        A sudoku in dictionary form
    Returns:
        solved sudoku in dictionary form or False
    """

    # First, reduce the puzzle using the previous function
    values = reduce_puzzle(values)
    if values is False:
        return False ## Failed earlier
    if all(len(values[s]) == 1 for s in boxes): 
        return values ## Solved!
    # Choose one of the unfilled squares with the fewest possibilities
    n,s = min((len(values[s]), s) for s in boxes if len(values[s]) > 1)
    # Now use recurrence to solve each one of the resulting sudokus, and 
    for value in values[s]:
        new_sudoku = values.copy()
        new_sudoku = assign_value(new_sudoku, s, value)
        attempt = search(new_sudoku)
        if attempt:
            return attempt


# In[11]:

def solve(grid):
    """
    Find the solution to a Sudoku grid.
    Args:
        grid(string): a string representing a sudoku grid.
            Example: '2.............62....1....7...6..8...3...9...7...6..4...4....8....52.............3'
    Returns:
        The dictionary representation of the final sudoku grid. False if no solution exists.
    """
        
    values = grid_values(grid)
    return search(values)


# In[12]:

if __name__ == '__main__':
    diag_sudoku_grid = '2.............62....1....7...6..8...3...9...7...6..4...4....8....52.............3'
    display(solve(diag_sudoku_grid))

    try:
        from visualize import visualize_assignments
        visualize_assignments(assignments)

    except SystemExit:
        pass
    except:
        print('We could not visualize your board due to a pygame issue. Not a problem! It is not a requirement.')


# In[ ]:



