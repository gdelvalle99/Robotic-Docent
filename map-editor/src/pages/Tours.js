import React, { useState, useEffect } from "react";
import { TourList } from "../components/TourList";
import { floorLink } from "../links";
import axios from "axios";
import { Fab } from "@material-ui/core";
import { Dialog } from "@material-ui/core";
import { DialogActions } from "@material-ui/core";
import { DialogContent } from "@material-ui/core";
import { DialogTitle } from "@material-ui/core";
import { Divider } from "@material-ui/core";
import { TextField } from "@material-ui/core";
import { Button } from "@material-ui/core";
import DeleteIcon from "@material-ui/icons/Delete";
import AddIcon from "@material-ui/icons/Add";
import NavigationIcon from "@material-ui/icons/Navigation";

const convertSimpleISODate = function (dateString) {
    if (dateString) {
        let d = new Date(dateString);

        return d.toISOString().substring(0, 10);
    } else {
        return null;
    }
};

export const Tours = () => {
    const [openModal, setOpenModal] = useState(false);
    const [openDeleteAlert, setOpenDeleteAlert] = useState(false);

    const handleClick = () => {
        console.log("ayo");
    };

    const createTour = () => {
        setOpenModal(true);
    };

    const handleCloseModal = () => {
        setOpenModal(false);
    };

    const handleCloseAlert = () => {
        setOpenDeleteAlert(false);
    };

    const handleOpenAlert = () => {
        setOpenDeleteAlert(true);
    };

    const handleDeleteClick = () => {
        console.log("delete");
    };

    const handleChange = () => {
        console.log("we change");
    };

    return (
        <div className="tours-page">
            <TourList />
            <div className="tours-buttons">
                <Fab
                    variant="extended"
                    size="large"
                    style={{ height: "auto", width: "400px", padding: "50px" }}
                    onClick={createTour}
                >
                    <AddIcon />
                    Create New Tour
                </Fab>
                <Fab
                    variant="extended"
                    size="large"
                    style={{ height: "auto", width: "400px", padding: "50px" }}
                    onClick={handleClick}
                >
                    <NavigationIcon />
                    Start Tour
                </Fab>
            </div>
            <Dialog
                open={openModal}
                onClose={handleCloseModal}
                fullWidth={true}
            >
                <DialogTitle>
                    <div className="tour-item-header">
                        <TextField
                            required={true}
                            className="tour-item-title"
                            placeholder="Title"
                            variant="outlined"
                            name="title"
                            onBlur={handleChange}
                            defaultValue=" "
                            fullWidth
                        />
                        <TextField
                            className="tour-item-subtitle"
                            placeholder="Subtitle"
                            size={"small"}
                            variant="outlined"
                            name="subtitle"
                            onBlur={handleChange}
                            defaultValue=" "
                            fullWidth
                        />
                        <Divider variant="middle" />
                    </div>
                </DialogTitle>
                <DialogContent>
                    <div className="tour-item-content-container">
                        <TextField
                            className="tour-item-description"
                            label="Description"
                            variant="outlined"
                            multiline
                            rows={5}
                            name="description"
                            onBlur={handleChange}
                            defaultValue=" "
                            fullWidth
                        />
                        <div className="tour-item-dates">
                            <TextField
                                className="tour-item-startdate"
                                label="Start Date"
                                variant="outlined"
                                type="date"
                                name="start_date"
                                onBlur={handleChange}
                                defaultValue={convertSimpleISODate(new Date())}
                                InputLabelProps={{ shrink: true }}
                                fullWidth
                            />
                            <TextField
                                className="tour-item-enddate"
                                label="End Date"
                                variant="outlined"
                                type="date"
                                name="end_date"
                                onBlur={handleChange}
                                defaultValue={convertSimpleISODate(new Date())}
                                InputLabelProps={{ shrink: true }}
                                fullWidth
                            />
                        </div>
                        <TextField
                            className="tour-item-theme"
                            label="Theme"
                            variant="outlined"
                            name="theme"
                            onBlur={handleChange}
                            defaultValue=" "
                            fullWidth
                        />
                    </div>
                </DialogContent>
                <DialogActions>
                    <div className="tour-delete-button">
                        <Button onClick={handleOpenAlert} variant="outlined">
                            <DeleteIcon fontSize="small" />
                        </Button>
                    </div>
                </DialogActions>
            </Dialog>
            <Dialog open={openDeleteAlert} onClose={handleCloseAlert}>
                <DialogTitle>Delete this tour?</DialogTitle>
                <DialogActions>
                    <div className="tour-delete-alert-buttons">
                        <div className="tour-delete-alert-button-delete">
                            <Button
                                onClick={handleDeleteClick}
                                variant="outlined"
                            >
                                Delete
                            </Button>
                        </div>
                        <div className="tour-delete-alert-button-cancel">
                            <Button
                                onClick={handleCloseAlert}
                                variant="outlined"
                            >
                                Cancel
                            </Button>
                        </div>
                    </div>
                </DialogActions>
            </Dialog>
        </div>
    );
};
