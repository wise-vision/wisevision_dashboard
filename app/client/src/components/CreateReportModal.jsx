import React, { useState, useEffect } from 'react';
import jsPDF from 'jspdf';
import '../styles/CreateReportModal.css';

const CreateReportModal = ({ isOpen, onClose }) => {
    const [topics, setTopics] = useState([]);
    const [selectedTopics, setSelectedTopics] = useState([]);
    const [reportName, setReportName] = useState('');

    useEffect(() => {
        if (isOpen) {
            console.log('Modal is open, fetching topics...');
            fetchTopics();
        }
    }, [isOpen]);

    // Fetch list of topics from backend
    const fetchTopics = async () => {
        try {
            console.log('Fetching topics from API...');
            const response = await fetch('http://localhost:5000/api/topics');
            const data = await response.json();
            console.log('Topics fetched successfully:', data);
            setTopics(data);
        } catch (error) {
            console.error('Error fetching topics:', error);
        }
    };

    const handleGenerateReport = async () => {
        try {
            console.log('Starting report generation...');
            // Fetch detailed data for each selected topic from the API
            const detailedDataPromises = selectedTopics.map(async (topic) => {
                console.log(`Fetching detailed data for topic: ${topic}`);
                const encodedTopic = encodeURIComponent(topic);
                const response = await fetch(
                    `http://localhost:5000/api/topic_echo_data_base_any/${encodedTopic}?type=lora_msgs/msg/E5BoardUplink`
                );
                const data = await response.json();
                console.log(`Data for topic "${topic}" fetched successfully.`, data);
                return {
                    topic,
                    data: data.messages || []
                };
            });

            const detailedData = await Promise.all(detailedDataPromises);
            console.log('All detailed data fetched successfully.');

            // Create a new PDF document
            const doc = new jsPDF();
            console.log('New PDF document created.');

            let yOffset = 20;
            const lineHeight = 10;
            const pageWidth = doc.internal.pageSize.getWidth();
            const margin = 10;
            const usableWidth = pageWidth - margin * 2;

            doc.setFontSize(8);

            // Generate the content for the PDF report
            detailedData.forEach(topicData => {
                console.log(`Adding data for topic: ${topicData.topic}`);

                // Add topic header
                doc.text(`Topic: ${topicData.topic}`, margin, yOffset);
                yOffset += lineHeight;

                topicData.data.forEach((message, index) => {
                    const messageText = `Message ${index + 1}: ${JSON.stringify(message)}`;
                    const splitText = doc.splitTextToSize(messageText, usableWidth); // Split text to fit within the usable width

                    splitText.forEach((line) => {
                        // Check if adding the next line exceeds page height
                        if (yOffset > doc.internal.pageSize.getHeight() - margin) {
                            console.log('Content exceeds page height, adding new page...');
                            doc.addPage();
                            yOffset = margin; // Reset yOffset after adding a new page
                        }
                        doc.text(line, margin, yOffset);
                        yOffset += lineHeight;
                    });
                });

                yOffset += lineHeight; // Space between topics
            });


            const fileName = reportName.trim() ? `${reportName}.pdf` : 'report.pdf';
            console.log(`Saving PDF document as: ${fileName}`);

            // Save the generated PDF
            doc.save(fileName);
            console.log('PDF document saved successfully.');

            onClose();
        } catch (error) {
            console.error('Error generating report:', error);
        }
    };

    if (!isOpen) return null;

    return (
        <div className="modal">
            <div className="modal-content">
                <h2>Generate Report</h2>

                {/* Report Name */}
                <div className="form-group">
                    <label>Report Name:</label>
                    <input
                        type="text"
                        value={reportName}
                        onChange={(e) => setReportName(e.target.value)}
                        placeholder="Enter report name"
                    />
                </div>

                {/* Topic Selection */}
                <div className="form-group">
                    <label>Select Topics:</label>
                    <select
                        multiple
                        value={selectedTopics}
                        onChange={(e) => setSelectedTopics(Array.from(e.target.selectedOptions, option => option.value))}
                    >
                        {topics.map((topic) => (
                            <option key={topic.id} value={topic.name}>
                                {topic.name}
                            </option>
                        ))}
                    </select>
                </div>

                {/* Actions */}
                <div className="modal-actions">
                    <button onClick={handleGenerateReport}>Generate Report</button>
                    <button onClick={onClose}>Cancel</button>
                </div>
            </div>
        </div>
    );
};

export default CreateReportModal;
