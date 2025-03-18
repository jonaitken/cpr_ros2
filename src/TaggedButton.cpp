#include "qt_includes.h"
#include "TaggedButton.h"

namespace cpr_ros2
{
    //! \brief Returns the tag that is associated with the button.
    //! \return The tag that is associated with the button.
    int TaggedButton::get_Tag() const
    {
        return m_Tag;
    }

    //! \brief Constructor of the TaggedButton class.
    //! \param tag The tag that will be associated with the button.
    //! \param pParent Pointer to the parent QWidget of the button.
    TaggedButton::TaggedButton(const int tag, QWidget* pParent) : 
        QPushButton(pParent),
        m_Tag(tag)
    {
        connect(this,&QAbstractButton::clicked,this,&TaggedButton::OnClicked);
    }

    //! \brief Destructor of the TaggedButton class.
    TaggedButton::~TaggedButton()
    {
    }

    //! \brief Handler for the clicked event of the base class.
    //! \param bChecked Flag indicating the checked-state of the button.
    void TaggedButton::OnClicked(bool bChecked)
    {
        emit buttonClicked(m_Tag,bChecked);
    }
}


#include "moc_TaggedButton.cpp"
