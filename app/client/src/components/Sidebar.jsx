import React, { useState } from 'react';
import addIcon from '../assets/images/addIcon.png';
import logo from '../assets/images/wiseVisionLogo.png';
import line from '../assets/images/lineImage.png';
import '../styles/Sidebar.css';
import CreateReportModal from './CreateReportModal';
import CreateActionModal from './CreateActionModal';
import DeleteActionModal from './DeleteActionModal'; // Import the new component

const Sidebar = ({ setIsModalOpen, setIsDeleteModalOpen, openActionsModal }) => {
    const [isReportModalOpen, setIsReportModalOpen] = useState(false);
    const [isActionModalOpen, setIsActionModalOpen] = useState(false);
    const [isDeleteActionModalOpen, setIsDeleteActionModalOpen] = useState(false); // State for delete action modal

    return (
        <div className="menu">
            <div className="logo">
                <img src={logo} alt="WiseVision Logo" className="logo--image"/>
            </div>
            <div className="line">
                <img src={line} alt="Line" className="logo--image"/>
            </div>
            <div className="menu--list">
                <button className="menu--item" onClick={() => setIsActionModalOpen(true)}>
                    <img src={addIcon} alt="Add Action" className="icon"/>
                    <span>Add Action</span>
                </button>
                <button className="menu--item" onClick={openActionsModal}>
                    <img src={addIcon} alt="Actions" className="icon"/>
                    <span>Actions</span>
                </button>
                <button className="menu--item" onClick={() => setIsReportModalOpen(true)}>
                    <img src={addIcon} alt="Create report" className="icon"/>
                    <span>Create report</span>
                </button>
                <button className="menu--item">
                    <img src={addIcon} alt="Export data" className="icon"/>
                    <span>Export data</span>
                </button>
                <button className="menu--item" onClick={() => setIsModalOpen(true)}>
                    <img src={addIcon} alt="Add new chart" className="icon"/>
                    <span>Add new chart</span>
                </button>
                {/* Button to open delete chart modal */}
                <button className="menu--item" onClick={() => setIsDeleteModalOpen(true)}>
                    <img src={addIcon} alt="Delete chart" className="icon"/>
                    <span>Delete chart</span>
                </button>
                {/* Button to open delete action modal */}
                <button className="menu--item" onClick={() => setIsDeleteActionModalOpen(true)}>
                    <img src={addIcon} alt="Delete action" className="icon"/>
                    <span>Delete action</span>
                </button>
            </div>

            {/* Modal for creating a report */}
            <CreateReportModal
                isOpen={isReportModalOpen}
                onClose={() => setIsReportModalOpen(false)}
            />

            {/* Modal for creating an action */}
            <CreateActionModal
                isOpen={isActionModalOpen}
                onClose={() => setIsActionModalOpen(false)}
            />

            {/* Modal for deleting an action */}
            <DeleteActionModal
                isOpen={isDeleteActionModalOpen}
                onClose={() => setIsDeleteActionModalOpen(false)}
            />
        </div>
    );
};

export default Sidebar;
