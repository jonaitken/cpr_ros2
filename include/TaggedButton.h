#pragma once

namespace cpr_ros2
{
	//! \class TaggedButton TaggedButton.h <TaggedButton.h>
	//! \brief A class for tagable push buttons.
    class TaggedButton: public QPushButton
    {
    	Q_OBJECT
    private:
        //! The tag associated with the button.
        const int m_Tag;
    public:
        int get_Tag() const;
        TaggedButton(const int tag, QWidget* pParent=nullptr);
        virtual ~TaggedButton();
    signals:
        //! \brief Signal that will fire when the button is clicked.
        //! \param tag The tag that is associated with the button that was clicked.
        //! \param bChecked Flag indicating the checked-state of the button that was clicked.
        void buttonClicked(int tag, bool bChecked=false);
    private:
        void OnClicked(bool bChecked=false);
    };
}
