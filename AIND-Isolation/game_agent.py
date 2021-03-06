
# coding: utf-8

# In[1]:

"""Finish all TODO items in this file to complete the isolation project, then
test your agent's strength against a set of known agents using tournament.py
and include the results in your report.
"""
import random


# In[2]:

class SearchTimeout(Exception):
    """Subclass base exception for code clarity. """
    pass


# In[3]:

def custom_score(game, player):
    """Calculate the heuristic value of a game state from the point of view
    of the given player.

    This should be the best heuristic function for your project submission.

    Note: this function should be called from within a Player instance as
    `self.score()` -- you should not need to call this function directly.

    Parameters
    ----------
    game : `isolation.Board`
        An instance of `isolation.Board` encoding the current state of the
        game (e.g., player locations and blocked cells).

    player : object
        A player instance in the current game (i.e., an object corresponding to
        one of the player objects `game.__player_1__` or `game.__player_2__`.)

    Returns
    -------
    float
        The heuristic value of the current game state to the specified player.

    Score Calculation
    -----------------
    # of available moves
    Maximize the number of open moves
    """
    return float(len(game.get_legal_moves(player=player)))


# In[4]:

def custom_score_2(game, player):
    """Calculate the heuristic value of a game state from the point of view
    of the given player.

    Note: this function should be called from within a Player instance as
    `self.score()` -- you should not need to call this function directly.

    Parameters
    ----------
    game : `isolation.Board`
        An instance of `isolation.Board` encoding the current state of the
        game (e.g., player locations and blocked cells).

    player : object
        A player instance in the current game (i.e., an object corresponding to
        one of the player objects `game.__player_1__` or `game.__player_2__`.)

    Returns
    -------
    float
        The heuristic value of the current game state to the specified player.

    Score Calculation
    -----------------
    -(# of available opponent moves)
    Maximize the number of blocked opponent moves
    """
    return float(-len((game.get_legal_moves(player=game.get_opponent(player)))))


# In[5]:

def custom_score_3(game, player):
    """Calculate the heuristic value of a game state from the point of view
    of the given player.

    Note: this function should be called from within a Player instance as
    `self.score()` -- you should not need to call this function directly.

    Parameters
    ----------
    game : `isolation.Board`
        An instance of `isolation.Board` encoding the current state of the
        game (e.g., player locations and blocked cells).

    player : object
        A player instance in the current game (i.e., an object corresponding to
        one of the player objects `game.__player_1__` or `game.__player_2__`.)

    Returns
    -------
    float
        The heuristic value of the current game state to the specified player.

    Score Calculation
    -----------------
    (weight_1)(# of available moves) - (weight_2)(# available opponent moves)
    Maximize the ratio of available moves to available opponent moves.
    Weight to control importance at various stages of game
    """
    #After running internal tournaments, found that weighting does not
    #improve perrformance when the weights are static. Commenting weights out,
    #but will revisit with a dynamic evaluation function
    #weight_1 = .67
    #weight_2 = .33
    player_legal_moves = float(len(game.get_legal_moves(player=player)))
    op_legal_moves = float(len(game.get_legal_moves(player=game.get_opponent(player))))
    #score = weight_1 * player_legal_moves - weight_2 * op_legal_moves
    score = player_legal_moves - op_legal_moves
    return score


# In[6]:

class IsolationPlayer:
    """Base class for minimax and alphabeta agents -- this class is never
    constructed or tested directly.

    ********************  DO NOT MODIFY THIS CLASS  ********************

    Parameters
    ----------
    search_depth : int (optional)
        A strictly positive integer (i.e., 1, 2, 3,...) for the number of
        layers in the game tree to explore for fixed-depth search. (i.e., a
        depth of one (1) would only explore the immediate sucessors of the
        current state.)

    score_fn : callable (optional)
        A function to use for heuristic evaluation of game states.

    timeout : float (optional)
        Time remaining (in milliseconds) when search is aborted. Should be a
        positive value large enough to allow the function to return before the
        timer expires.
    """
    def __init__(self, search_depth=3, score_fn=custom_score_3, timeout=10.):
        self.search_depth = search_depth
        self.score = score_fn
        self.time_left = None
        self.TIMER_THRESHOLD = timeout


# In[7]:

class MinimaxPlayer(IsolationPlayer):
    """Game-playing agent that chooses a move using depth-limited minimax
    search. You must finish and test this player to make sure it properly uses
    minimax to return a good move before the search time limit expires.
    """

    def get_move(self, game, time_left):
        """Search for the best move from the available legal moves and return a
        result before the time limit expires.

        **************  YOU DO NOT NEED TO MODIFY THIS FUNCTION  *************

        For fixed-depth search, this function simply wraps the call to the
        minimax method, but this method provides a common interface for all
        Isolation agents, and you will replace it in the AlphaBetaPlayer with
        iterative deepening search.

        Parameters
        ----------
        game : `isolation.Board`
            An instance of `isolation.Board` encoding the current state of the
            game (e.g., player locations and blocked cells).

        time_left : callable
            A function that returns the number of milliseconds left in the
            current turn. Returning with any less than 0 ms remaining forfeits
            the game.

        Returns
        -------
        (int, int)
            Board coordinates corresponding to a legal move; may return
            (-1, -1) if there are no available legal moves.
        """
        self.time_left = time_left

        # Initialize the best move so that this function returns something
        # in case the search fails due to timeout
        best_move = (-1, -1)

        try:
            # The try/except block will automatically catch the exception
            # raised when the timer is about to expire.
            return self.minimax(game, self.search_depth)

        except SearchTimeout:
            pass  # Handle any actions required after timeout as needed

        # Return the best move from the last completed search iteration
        return best_move

    def minimax(self, game, depth):
        """Implement depth-limited minimax search algorithm as described in
        the lectures.

        This should be a modified version of MINIMAX-DECISION in the AIMA text.
        https://github.com/aimacode/aima-pseudocode/blob/master/md/Minimax-Decision.md

        **********************************************************************
            You MAY add additional methods to this class, or define helper
                 functions to implement the required functionality.
        **********************************************************************

        Parameters
        ----------
        game : isolation.Board
            An instance of the Isolation game `Board` class representing the
            current game state

        depth : int
            Depth is an integer representing the maximum number of plies to
            search in the game tree before aborting

        Returns
        -------
        (int, int)
            The board coordinates of the best move found in the current search;
            (-1, -1) if there are no legal moves

        Notes
        -----
            (1) You MUST use the `self.score()` method for board evaluation
                to pass the project tests; you cannot call any other evaluation
                function directly.

            (2) If you use any helper functions (e.g., as shown in the AIMA
                pseudocode) then you must copy the timer check into the top of
                each helper function or else your agent will timeout during
                testing.
        """
        if self.time_left() < self.TIMER_THRESHOLD:
            raise SearchTimeout()

        def terminal_test(game, depth):
            """ Return True if the game is over for the active player
            and False otherwise.
            """
            if self.time_left() < self.TIMER_THRESHOLD:
                raise SearchTimeout()
            return (not bool(game.get_legal_moves())) or (depth<=0)


        def min_value(game, depth):
            """ Return the value for a win (+1) if the game is over,
            otherwise return the minimum value over all legal child
            nodes.
            """
            if terminal_test(game, depth):
                #print("min depth {0}".format(depth))
                return self.score(game, self)
            v = float("inf")
            for move in game.get_legal_moves():
                v = min(v, max_value(game.forecast_move(move), depth-1))
            return v


        def max_value(game, depth):
            """ Return the value for a loss (-1) if the game is over,
            otherwise return the maximum value over all legal child
            nodes.
            """
            if terminal_test(game, depth):
                #print("max depth {0}".format(depth))
                return self.score(game, self)
            v = float("-inf")
            for move in game.get_legal_moves():
                v = max(v, min_value(game.forecast_move(move), depth-1))
            return v

        """ Return the move along a branch of the game tree that
        has the best possible value.  A move is a pair of coordinates
        in (column, row) order corresponding to a legal move for
        the searching player.

        ignore the special case of calling this function
        from a terminal state.
        """
        #init min values
        v = float("-inf")
        best_move, best_score = (-1,-1), v
        if not game.get_legal_moves():
            return best_move
        for move in game.get_legal_moves():
            v = min_value(game.forecast_move(move), depth-1)
            #track best move and score
            if v > best_score:
                best_move, best_score = move, v
                #print(game.to_string())
        return best_move


# In[8]:


# from agent_test import *

# test1 = IsolationTest(MinimaxPlayer(),MinimaxPlayer())
# test1.play()


# In[9]:

class AlphaBetaPlayer(IsolationPlayer):
    """Game-playing agent that chooses a move using iterative deepening minimax
    search with alpha-beta pruning. You must finish and test this player to
    make sure it returns a good move before the search time limit expires.
    """

    def get_move(self, game, time_left):
        """Search for the best move from the available legal moves and return a
        result before the time limit expires.

        Modify the get_move() method from the MinimaxPlayer class to implement
        iterative deepening search instead of fixed-depth search.

        **********************************************************************
        NOTE: If time_left() < 0 when this function returns, the agent will
              forfeit the game due to timeout. You must return _before_ the
              timer reaches 0.
        **********************************************************************

        Parameters
        ----------
        game : `isolation.Board`
            An instance of `isolation.Board` encoding the current state of the
            game (e.g., player locations and blocked cells).

        time_left : callable
            A function that returns the number of milliseconds left in the
            current turn. Returning with any less than 0 ms remaining forfeits
            the game.

        Returns
        -------
        (int, int)
            Board coordinates corresponding to a legal move; may return
            (-1, -1) if there are no available legal moves.
        """
        self.time_left = time_left

        # Initialize the best move so that this function returns something
        # in case the search fails due to timeout
        best_move = (-1, -1)

        try:
            # The try/except block will automatically catch the exception
            # raised when the timer is about to expire.
            depth = 1
            while 1:
                best_move = self.alphabeta(game, depth)
                depth+=1
            return best_move

        except SearchTimeout:
            pass  # Handle any actions required after timeout as needed

        # Return the best move from the last completed search iteration
        return best_move

    def alphabeta(self, game, depth, alpha=float("-inf"), beta=float("inf")):
        """Implement depth-limited minimax search with alpha-beta pruning as
        described in the lectures.

        This should be a modified version of ALPHA-BETA-SEARCH in the AIMA text
        https://github.com/aimacode/aima-pseudocode/blob/master/md/Alpha-Beta-Search.md

        **********************************************************************
            You MAY add additional methods to this class, or define helper
                 functions to implement the required functionality.
        **********************************************************************

        Parameters
        ----------
        game : isolation.Board
            An instance of the Isolation game `Board` class representing the
            current game state

        depth : int
            Depth is an integer representing the maximum number of plies to
            search in the game tree before aborting

        alpha : float
            Alpha limits the lower bound of search on minimizing layers

        beta : float
            Beta limits the upper bound of search on maximizing layers

        Returns
        -------
        (int, int)
            The board coordinates of the best move found in the current search;
            (-1, -1) if there are no legal moves

        Notes
        -----
            (1) You MUST use the `self.score()` method for board evaluation
                to pass the project tests; you cannot call any other evaluation
                function directly.

            (2) If you use any helper functions (e.g., as shown in the AIMA
                pseudocode) then you must copy the timer check into the top of
                each helper function or else your agent will timeout during
                testing.
        """
        if self.time_left() < self.TIMER_THRESHOLD:
            raise SearchTimeout()

        def terminal_test(game, depth):
            """ Return True if the game is over for the active player
            and False otherwise.
            """
            if self.time_left() < self.TIMER_THRESHOLD:
                raise SearchTimeout()
            return (not bool(game.get_legal_moves())) or (depth <= 0)

        def min_value(game, depth, alpha, beta):
            """ Return the value for a win (+1) if the game is over,
            otherwise return the minimum value over all legal child
            nodes.
            """
            v = float("inf")
            if terminal_test(game, depth):
                return self.score(game, self)
            for move in game.get_legal_moves():
                v = min(v, max_value(game.forecast_move(move), depth-1, alpha, beta))
                if v<= alpha:
                    return v
                beta = min(beta, v)
            return v



        def max_value(game, depth, alpha, beta):
            """ Return the value for a loss (-1) if the game is over,
            otherwise return the maximum value over all legal child
            nodes.
            """
            v = float("-inf")
            if terminal_test(game, depth):
                return self.score(game, self)
            for move in game.get_legal_moves():
                v = max(v, min_value(game.forecast_move(move), depth-1, alpha, beta))
                if v>= beta:
                    return v
                alpha = max(alpha, v)
            return v

        """ Return the move along a branch of the game tree that
        has the best possible value.  A move is a pair of coordinates
        in (column, row) order corresponding to a legal move for
        the searching player.

        ignore the special case of calling this function
        from a terminal state.
        """
        #init min values
        v = float("-inf")
        best_move, best_score = (-1,-1), v
        if not bool(game.get_legal_moves()):
            return best_move
        for move in game.get_legal_moves():
            v = min_value(game.forecast_move(move), depth-1, alpha, beta)
            alpha = max(alpha, v)
            #track best move and score
            if v > best_score:
                best_move, best_score = move, v
        return best_move


# In[10]:

# test2 = IsolationTest(AlphaBetaPlayer(), MinimaxPlayer())
# test2.play()
