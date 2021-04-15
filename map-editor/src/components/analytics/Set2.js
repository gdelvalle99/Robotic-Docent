import React from "react";
import OutlinedCard from "./OutlinedCard";

export default function Set2() {
    const body = (one, two) => {
        return (
            <div>
                {one}
                <br />
                {two}
            </div>
        );
    };
    return (
        <div
            style={{
                display: "flex",
                justifyContent: "space-between",
                flexDirection: "column",
            }}
        >
            <div
                style={{
                    display: "flex",
                    justifyContent: "space-between",
                }}
            >
                <OutlinedCard
                    title="Robot Battery Level"
                    fact={"89%"}
                    disableAction
                />
                <OutlinedCard
                    title="Number of Error States"
                    fact={63}
                    subfact="Times where the robot was stuck"
                    body={body("Mainly found in in:", "the Rock Exhibit")}
                />
            </div>
            <div
                style={{
                    display: "flex",
                    justifyContent: "space-between",
                }}
            >
                <OutlinedCard
                    title="Total Number of Tours"
                    fact={105}
                    subfact="Tours by the robot"
                    body={body("Most Popular Day:", "Saturday")}
                />
                <OutlinedCard
                    title="Shortest Tour Length"
                    fact={"13 Minutes 32 Seconds"}
                    // subfact=""
                    // body={body("Where is the bathroom?")}
                    disableAction
                />
            </div>
        </div>
    );
}
