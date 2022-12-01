/* Implement a variety of methods for operations on the book module that
 * mutate the data saved into chapters.
 * Chapter creation and relations are done when a csv file is read.
 * These functions are called with pre-handled parameters (utils.hh and cli).
 * For further info on all the possible calls and csv file handling check the
 * interface module files cli.hh and cli.cpp.
 *
 *
 * For documentation on the below methods chech the header file (book.hh).
 *
 *
 * */

#include "book.hh"

Book::Book()
{}

Book::~Book()
{
    // deletes every chapter pointer in the storage vector
    for(Chapter* chapters : chapterVec)
    {
        delete chapters;
    }
}

void Book::addNewChapter(const std::string &id, const std::string &fullName,
                         int length)
{
    // Add every line found in file if its not present in the container
    for(Chapter* chapters : chapterVec)
    {
        if (chapters -> id_ == id)
        {
            std::cout << "Error: Already exists." << std::endl;
            return;
        }
    }
    Chapter* chapter = new Chapter;
    chapter -> id_ = id;
    chapter -> fullName_ = fullName;
    chapter -> length_ = length;
    chapterVec.push_back(chapter);
}

void Book::addRelation(const std::string &subchapter,
                       const std::string &parentChapter)
{
    for(Chapter* parentChapters : chapterVec)
    {
        for(Chapter* subchapters : chapterVec)
        {
            if(parentChapters -> id_ == parentChapter &&
                    subchapters -> id_ == subchapter)
            {
                subchapters -> parentChapter_ = parentChapters;
                parentChapters -> subchapters_.push_back(subchapters);
            }
        }
    }
}

// Why is params here and some other methods when there is nothing important?
// Could be removed but it is part of the public interface and alterations
// are not allowed. Removed warnings with useless code.
void Book::printIds(Params params) const
{
    if(params.size()==0)
    {}
    // Add id and fullname to map so alphabetical sort
    // for printing is done automatically.
    std::map< std::string, std::string> mapToPrint;
    for(Chapter* chapters : chapterVec)
    {
        std::string id = chapters -> id_;
        std::string fullname = chapters -> fullName_;
        mapToPrint.insert(std::make_pair(fullname, id));
    }

    std::cout << "Book has "  << mapToPrint.size() << " chapters:" << std::endl;
    for(auto& itr : mapToPrint)
    {
        std::cout << itr.first << " --- " << itr.second << std::endl;
    }
}

void Book::printContents(Params params) const
{
    if(params.size()==0)
    {}
    int indent = 1;
    int rowNum = 1;
    for(Chapter* chapters : chapterVec)
    {
        if(chapters -> parentChapter_ == nullptr)
        {
            if(chapters -> isOpen_ == true)
            {
                std::cout << "-";
                printChapter(chapters, indent, rowNum);
                printRecursive(chapters -> subchapters_, indent, rowNum);
            }
            else
            {
                std::cout << "+";
                printChapter(chapters, indent, rowNum);
            }
            ++rowNum;
        }
    }
}

void Book::printRecursive(const std::vector<Chapter *> chapterVec,
                          int indent, int rowNum) const
{
    rowNum = 1;
    indent = indent + 2;
    if(chapterVec.size() != 0)
    {
        for(Chapter* chapters : chapterVec)
        {
            if(chapters -> isOpen_ == true)
            {
                std::cout << "-";
                printChapter(chapters, indent, rowNum);
                printRecursive(chapters -> subchapters_, indent, rowNum);
            }
            else
            {
                std::cout << "+";
                printChapter(chapters, indent, rowNum);
            }
            ++rowNum;
        }
    }
    return;
}

void Book::printChapter(const Chapter* chapter, int indent, int rowNum) const
{
    std::cout << std::string(indent, ' ') << rowNum << ". ";
    std::cout << chapter -> fullName_ << " ( " << chapter -> length_;
    std::cout << " )" << std::endl;

}

void Book::close(Params params) const
{

    Chapter* chapterToClose = findChapter(params.at(0));
    if(chapterToClose != nullptr)
    {
        chapterToClose -> isOpen_ = false;
        if(chapterToClose->subchapters_.size() != 0)
        {
            // Call to close all the subchapters except ones that have no
            // further subchapters
            recursiveClose(chapterToClose->subchapters_);
        }
    }
}

void Book::recursiveClose(const std::vector<Chapter *> chapterVec) const
{
    for(Chapter* chapters : chapterVec)
    {
        if(chapters -> subchapters_.size() != 0)
        {
            chapters -> isOpen_ = false;
            recursiveClose(chapters -> subchapters_);
        }
        else
        {
            chapters -> isOpen_ = true;
        }
    }
}


void Book::open(Params params) const
{
    Chapter* chapterToClose = findChapter(params.at(0));
    if(chapterToClose != nullptr)
    {
        chapterToClose -> isOpen_ = true;
    }
}

void Book::openAll(Params params) const
{
    if(params.size()==0)
    {}
    for(Chapter* chapters : chapterVec)
    {
        chapters -> isOpen_ = true;
    }
}

void Book::printParentsN(Params params) const
{
    std::string group = "parent chapters";
    Chapter* chapter = findChapter(params.at(0));
    std::string id = params.at(0);
    int limit = stoi(params.at(1));
    std::vector<Chapter *> container;
    if(chapter != nullptr && checkNumValidity(limit))
    {
        int levelsUp = 0;
        // Walk chapter structs upstream until limit is reaced.
        while(levelsUp != limit)
        {
            if(chapter -> parentChapter_ != nullptr)
            {
                container.push_back(chapter -> parentChapter_);
                chapter = chapter -> parentChapter_;
            }
            ++levelsUp;
        }
        // Turn pointers into a vector of string ids.
        // Use general print to show wanted info.
        IdSet idsContainer = vectorToIdSet(container);
        printGroup(id, group, idsContainer);
    }
}

void Book::printSubchaptersN(Params params) const
{
    std::string group = "subchapters";
    Chapter* chapter = findChapter(params.at(0));
    std::string id = params.at(0);
    int limit = stoi(params.at(1));
    std::vector<Chapter *> container;

    if(chapter != nullptr && checkNumValidity(limit))
    {
        if(limit == 1)
        {
            container = chapter -> subchapters_;
        }
        // Init reference variables for recursion
        int levelDown = 1;
        std::vector<Chapter *> vec = {};

        container = getSubchaptersN(chapter -> subchapters_, limit,
                                    levelDown, vec);

        IdSet idsContainer = vectorToIdSet(container);
        printGroup(id, group, idsContainer);
    }
}

std::vector<Chapter *> Book::getSubchaptersN(std::vector<Chapter *> subchapters,
                                             int limit, int& levelDown,
                                             std::vector<Chapter *> &vec) const
{
    // Add subchapters to reference vector until limit is reached.
    for(Chapter* chapters : subchapters)
    {
        if(chapters -> subchapters_.size() != 0)
        {
            vec.push_back(chapters);
            if(levelDown != limit)
            {
                ++levelDown;
                getSubchaptersN(chapters ->subchapters_, limit, levelDown, vec);
            }
        }
        else
        {
            if(levelDown <= limit)
            {
                vec.push_back(chapters);
            }
        }
    }
    return vec;
}

bool Book::checkNumValidity(int num) const
{
    if(num < 1)
    {
        std::cout << "Error. Level can't be less than 1." << std::endl;
        return false;
    }
    return true;
}

void Book::printSiblingChapters(Params params) const
{
    std::string group = "sibling chapters";
    std::vector<Chapter *> container;
    Chapter* chapter = findChapter(params.at(0));
    std::string id = params.at(0);
    if(chapter != nullptr)
    {
        if(chapter -> parentChapter_ != nullptr)
        {
            Chapter* parentChapter = chapter -> parentChapter_;
            container = parentChapter -> subchapters_;
            int i =  0;
            // Remove chapter from being itselfs sibling.
            for(Chapter* chapters : container)
            {
                if(chapters -> id_ == chapter -> id_)
                {
                    container.erase(container.begin()+i);
                }
                ++i;
            }
        }
        IdSet idsContainer = vectorToIdSet(container);
        printGroup(id, group, idsContainer);
    }
}

void Book::printTotalLength(Params params) const
{
    Chapter* chapter = findChapter(params.at(0));
    int length = 0;
    int start = 0;
    std::string id = params.at(0);
    if(chapter != nullptr)
    {
        start += chapter -> length_;
        length = recursiveLength(chapter -> subchapters_, start);
        std::cout << "Total length of " << id << " is ";
        std::cout << length << "." << std::endl;
    }
}

int Book::recursiveLength(const std::vector<Chapter *> chapterVec,
                          int& num) const
{
    // Add to total num with recursion.
    for(Chapter* chapters : chapterVec)
    {
        if(chapters -> subchapters_.size() != 0)
        {
            num += chapters -> length_;
            recursiveLength(chapters -> subchapters_, num);
        }
        else
        {
            // When can't advance further.
            num += chapters -> length_;
        }
    }
    return num;
}

void Book::printLongestInHierarchy(Params params) const
{
    Chapter* chapter = findChapter(params.at(0));
    std::string id = params.at(0);
    chooseLengthType(chapter, id, false);
}

void Book::printShortestInHierarchy(Params params) const
{
    Chapter* chapter = findChapter(params.at(0));
    std::string id = params.at(0);
    chooseLengthType(chapter, id, true);
}

void Book::chooseLengthType(Chapter* chapter, std::string id,
                            bool shortest) const
{
    // numMap creation same for both lenghtInHierarchy methods
    std::map<std::string, int> numMap;
    if(chapter != nullptr)
    {
        numMap.insert(std::make_pair(chapter->id_, chapter -> length_));
        std::map<std::string, int> completeNumMap =
                getAllLengths(chapter -> subchapters_, numMap);

        // Call data print to process numMap by shortest or longest.
        if (shortest)
        {
            printLengthData(completeNumMap, id, "shortest");
        }
        else
        {
            printLengthData(completeNumMap, id, "longest");
        }
    }
}

std::map<std::string, int> Book::getAllLengths(const std::vector<
                                               Chapter *> chapterVec, std::map<
                                               std::string, int>& numMap) const
{
    // Mutate numMap with recursion.
    for(Chapter* chapters : chapterVec)
    {
        if(chapters -> subchapters_.size() != 0)
        {
            numMap.insert(std::make_pair(chapters->id_, chapters-> length_));
            getAllLengths(chapters -> subchapters_, numMap);
        }
        // Last subchapters are reached.
        else
        {
            numMap.insert(std::make_pair(chapters->id_, chapters-> length_));
        }
    }
    return numMap;
}

void Book::printLengthData(const std::map<std::string, int>& numMap,
                           std::string id, std::string type) const
{
    std::string numId = numMap.begin() -> first;
    int num = numMap.begin() -> second;
    //Looking for biggest length.
    if(type=="longest")
    {
        for(auto itr = numMap.begin(); itr != numMap.end(); ++itr)
        {
            if(itr -> second >= num)
            {
                num = itr -> second;
                numId = itr -> first;
            }
        }
    }
    // Shortest Length.
    else
    {
        for(auto itr = numMap.begin(); itr != numMap.end(); ++itr)
        {
            if(itr -> second <= num)
            {
                num = itr -> second;
                numId = itr -> first;
            }
        }
    }
    if(numId == id)
    {
        std::cout << "With the length of " << num << ", " << id << " is the ";
        std::cout << type << " chapter in their hierarchy." << std::endl;
    }
    else
    {
        std::cout << "With the length of " << num << ", " <<numId<< " is the ";
        std::cout << type << " chapter in " << id << "'s ";
        std::cout << "hierarchy." << std::endl;
    }
}

Chapter *Book::findChapter(const std::string &id) const
{
    Chapter* chapter = nullptr;
    for(Chapter* chapters : chapterVec)
    {
        if (chapters -> id_ == id)
        {
            chapter = chapters;
        }
    }
    if(chapter == nullptr)
    {
        std::cout << "Error: Not found: " << id << std::endl;
    }
    return chapter;
}

void Book::printGroup(const std::string &id, const std::string &group,
                      const IdSet &container) const
{
    if(container.size() != 0)
    {
        std::cout << id << " has " << container.size() << " " << group;
        std::cout << ":" << std::endl;
        // Set is in alphabetical order
        for(auto& ids : container)
        {
            std::cout << ids << std::endl;
        }
    }
    else
    {
        std::cout << id << " has no " << group << "." << std::endl;
    }
}

IdSet Book::vectorToIdSet(const std::vector<Chapter *> &container) const
{
    IdSet idsToPrint;
    for(Chapter* chapters : container)
    {
        std::string id = chapters -> id_;
        idsToPrint.insert(id);
    }
    return idsToPrint;
}
