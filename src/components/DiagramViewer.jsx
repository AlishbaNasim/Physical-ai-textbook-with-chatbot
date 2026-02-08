import React, { useState } from 'react';
import './DiagramViewer.css'; // Import the CSS file

const DiagramViewer = ({ src, alt, caption, title, children }) => {
  const [isExpanded, setIsExpanded] = useState(false);
  const [zoomLevel, setZoomLevel] = useState(1);

  const handleExpand = () => {
    setIsExpanded(!isExpanded);
  };

  const handleZoomIn = () => {
    setZoomLevel(Math.min(zoomLevel + 0.2, 3)); // Max 3x zoom
  };

  const handleZoomOut = () => {
    setZoomLevel(Math.max(zoomLevel - 0.2, 0.5)); // Min 0.5x zoom
  };

  const handleResetZoom = () => {
    setZoomLevel(1);
  };

  const handleDoubleClick = () => {
    setIsExpanded(!isExpanded);
  };

  return (
    <div className="diagram-viewer-container">
      <figure className={`diagram-figure ${isExpanded ? 'expanded' : ''}`}>
        <div
          className="diagram-wrapper"
          onClick={handleExpand}
          onDoubleClick={handleDoubleClick}
        >
          <img
            src={src}
            alt={alt || caption || title}
            className="diagram-image"
            style={{ transform: `scale(${zoomLevel})`, transformOrigin: 'center' }}
          />
        </div>

        {(caption || title) && (
          <figcaption className="diagram-caption">
            <strong>{title}</strong>: {caption}
          </figcaption>
        )}

        <div className="diagram-controls">
          <button
            onClick={handleExpand}
            className="control-button"
            title={isExpanded ? "Collapse diagram" : "Expand diagram"}
            aria-label={isExpanded ? "Collapse diagram" : "Expand diagram"}
          >
            {isExpanded ? "Collapse" : "Expand"}
          </button>

          {isExpanded && (
            <>
              <button
                onClick={handleZoomIn}
                className="control-button"
                title="Zoom in"
                aria-label="Zoom in"
              >
                +
              </button>

              <button
                onClick={handleZoomOut}
                className="control-button"
                title="Zoom out"
                aria-label="Zoom out"
              >
                -
              </button>

              <button
                onClick={handleResetZoom}
                className="control-button"
                title="Reset zoom"
                aria-label="Reset zoom"
              >
                Reset
              </button>
            </>
          )}
        </div>
      </figure>

      {isExpanded && children && (
        <div className="diagram-details">
          {children}
        </div>
      )}
    </div>
  );
};

// Simple diagram component for inline SVG diagrams
const InlineDiagram = ({ title, children, className = "" }) => {
  return (
    <div className={`inline-diagram ${className}`}>
      <h4>{title}</h4>
      <div className="diagram-content">
        {children}
      </div>
    </div>
  );
};

export { DiagramViewer, InlineDiagram };
export default DiagramViewer;