/* Mastermind
 *
 * Desc:
 *  A game where firstly a secret series of four colors is either given
 *  or generated. There can be identical colors, but the series will
 *  always have four positions. The user will have to guess this secret
 *  by inputting a series of four colors. After submitting a sensible input
 *  the program shows the guessed series and after that how many colors were
 *  totally correct (correct color in correct positions) number on the left
 *  side. On the right side the program shows how many colors were correct
 *  but had incorrect positioning. The user has 10 tries to guess the
 *  series correctly. The user wins a game by guessing the position of all
 *  the four colors right in the secret series.
 *  The program tells the player whether they have won or lost.
 * */

#include <colorseries.hh>
#include <iostream>
#include <vector>
#include <random>

using namespace std;

// Maximum number of guesses.
const unsigned int GUESS_MAX = 10;

// Number of colors in a series.
const unsigned int SIZE = 4;

// Length of the suffix part when printing a row.
const unsigned int SUFFIX_LENGTH_IN_PRINT = 5;

// Text printed at the beginning of the program.
const string INFO_TEXT = "Colors in use: \
B = Blue, R = Red, Y = Yellow, G = Green, O = Orange, V = Violet";

// Accepted colors in a string.
const string ACCEPTED_COLORS = "BRYGOV";

/* Checks if the user input is valid and also mutates the input to
 * uppercase to ease further processing.
 *
 * @param colors a string with 4 letters corresponding to the viable colors.
 * @return true if accepted, else false.
 * */
bool check_user_input(string& colors)
{
    if(colors.size() != SIZE) {
        cout << "Wrong size" << endl;
        return false;
    }
    for (unsigned long i=0; i < colors.size(); ++i) {
        colors.at(i) = toupper(colors.at(i));
        if (ACCEPTED_COLORS.find(colors.at(i)) == std::string::npos) {
            cout << "Unknown color" << endl;
            return false;
        }
    }
    return true;
}

/* Creates and adds a color series to a container.
 *
 * @param series_container reference to the container object where the
 *        series are to be saved.
 * @param colors a string with 4 letters corresponding to the viable colors.
 *        The ColorSeries object is created with the chars from the string.
 * @param compare_during_init by default false. If called with true will
 *        compare the newly created ColorSeries to the secret.
 * */
void add_color_series_to_container(ColorSeriesContainer& series_container,
                                   string colors, bool compare_during_init = false)
{
    ColorSeries serie(colors);
    if (compare_during_init)
    {
        // ColorSeries object used as a secret is always created and stored
        // first. That's why this is hard coded.
        serie.compare_to_secret(series_container.at(0));
    }
    series_container.push_back(serie);
}


/* Generates and adds the secret color series for the user to guess.
 *
 * @param series_container reference to the container where the secret
 *        should be stored.
 * */
void generate_secret_series(ColorSeriesContainer& series_container)
{
    cout << "Enter a seed value: ";
    int seed = 0;
    cin >> seed;
    std::default_random_engine rand_gen;
    rand_gen.seed(seed);
    std::uniform_int_distribution<int> distribution(0, 5);
    string colors = "";
    for(unsigned int i=0; i < SIZE; ++i)
    {
        colors.push_back(ACCEPTED_COLORS.at(distribution(rand_gen)));
    }
    add_color_series_to_container(series_container, colors);
}


/* Checks user inputs and calls functions based on it.
 *
 * @param series_container reference to the container for ColorSeries.
 *        Passed further to functions.
 * */
void get_input(ColorSeriesContainer& series_container)
{
    cout << "Enter an input way (R = random, L = list): ";
    string input_str = "";
    cin >> input_str;
    if(input_str == "R" or input_str == "r")
    {
        generate_secret_series(series_container);
    }
    else if(input_str == "L" or input_str == "l")
    {
        bool accepted = false;
        while(not accepted)
        {
            cout << "Enter four colors (four letters without spaces): ";
            string colors = "";
            cin >> colors;
            if (check_user_input(colors))
            {
                add_color_series_to_container(series_container, colors);
                accepted = true;
            }
        }
    }
    else
    {
        cout << "Bad input" << endl;
        get_input(series_container);
    }
}

// Prints a line consisting of the given character c.
// The length of the line is given in the parameter line_length.
void print_line_with_char(char c, unsigned int line_length)
{
    for(unsigned int i = 0; i < line_length; ++i)
    {
        cout << c;
    }
    cout << endl;
}

/* Shows user the status of the game. Calls functions
 * that print info and checks whether a win has happened.
 *
 * @param series_container reference to the container
 *        where all the ColorSeries are stored.
 * */
bool game_status(ColorSeriesContainer& series_container)
{
    bool victory = false;
    print_line_with_char('=', 2 * (SIZE + SUFFIX_LENGTH_IN_PRINT) + 1);
    for(unsigned long i=1; i < series_container.size(); ++i)
    {
        ColorSeries secret_serie = series_container.at(0);
        ColorSeries guess_serie = series_container.at(i);
        guess_serie.print_info();
        victory = guess_serie.check_score();
    }
    print_line_with_char('=', 2 * (SIZE + SUFFIX_LENGTH_IN_PRINT) + 1);
    return victory;
}

// Implements the actual game loop, where user-given guesses are read
// and compared to the secret row.
// On each round, all rows given so far are printed.
int main()
{
    cout << INFO_TEXT << endl;
    print_line_with_char('*', INFO_TEXT.size());
    ColorSeriesContainer series_container;
    get_input(series_container);

    unsigned int retries = 0;
    while(retries < GUESS_MAX)
    {
        cout << "ROW> ";
        string input_string = "";
        cin >> input_string;
        if (input_string == "q")
        {
            break;
        }
        if (check_user_input(input_string))
        {
            add_color_series_to_container(series_container, input_string, true);
            bool win = game_status(series_container);
            retries += 1;
            if (win)
            {
                cout << "You won: Congratulations!" << endl;
                break;
            }
        }
        continue;

    }
    if(retries == 10)
    {
        cout << "You lost: Maximum number of guesses done" << endl;
    }
    return 0;
}

